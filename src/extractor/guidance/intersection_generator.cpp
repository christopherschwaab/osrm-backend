#include "extractor/guidance/intersection_generator.hpp"

#include "extractor/geojson_debug_policies.hpp"
#include "util/geojson_debug_logger.hpp"

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/mergable-roads.hpp"
#include "extractor/guidance/toolkit.hpp"

#include "util/guidance/toolkit.hpp"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <utility>

#include <boost/range/algorithm/count_if.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

IntersectionGenerator::IntersectionGenerator(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const RestrictionMap &restriction_map,
    const std::unordered_set<NodeID> &barrier_nodes,
    const std::vector<QueryNode> &node_info_list,
    const CompressedEdgeContainer &compressed_edge_container,
    const util::NameTable &name_table_,
    const SuffixTable &street_name_suffix_table_)
    : node_based_graph(node_based_graph), restriction_map(restriction_map),
      barrier_nodes(barrier_nodes), node_info_list(node_info_list),
      coordinate_extractor(node_based_graph, compressed_edge_container, node_info_list),
      name_table(name_table_), street_name_suffix_table(street_name_suffix_table_)
{
}

Intersection IntersectionGenerator::operator()(const NodeID from_node, const EdgeID via_eid) const
{
    auto intersection = GetConnectedRoads(from_node, via_eid);
    const auto node_at_intersection = node_based_graph.GetTarget(via_eid);
    return AdjustForJoiningRoads(
        node_at_intersection, MergeSegregatedRoads(node_at_intersection, std::move(intersection)));
}

//                                               a
//                                               |
//                                               |
//                                               v
// For an intersection from_node --via_edi--> turn_node ----> c
//                                               ^
//                                               |
//                                               |
//                                               b
// This functions returns _all_ turns as if the graph was undirected.
// That means we not only get (from_node, turn_node, c) in the above example
// but also (from_node, turn_node, a), (from_node, turn_node, b). These turns are
// marked as invalid and only needed for intersection classification.
Intersection IntersectionGenerator::GetConnectedRoads(const NodeID from_node,
                                                      const EdgeID via_eid) const
{
    Intersection intersection;
    const NodeID turn_node = node_based_graph.GetTarget(via_eid);
    const NodeID only_restriction_to_node = [&]() {
        // If only restrictions refer to invalid ways somewhere far away, we rather ignore the
        // restriction than to not route over the intersection at all.
        const auto only_restriction_to_node =
            restriction_map.CheckForEmanatingIsOnlyTurn(from_node, turn_node);
        if (only_restriction_to_node != SPECIAL_NODEID)
        {
            // check if we can find an edge in the edge-rage
            for (const auto onto_edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
                if (only_restriction_to_node == node_based_graph.GetTarget(onto_edge))
                    return only_restriction_to_node;
        }
        // Ignore broken only restrictions.
        return SPECIAL_NODEID;
    }();
    const bool is_barrier_node = barrier_nodes.find(turn_node) != barrier_nodes.end();

    bool has_uturn_edge = false;
    bool uturn_could_be_valid = false;
    const util::Coordinate turn_coordinate = node_info_list[turn_node];

    const auto intersection_lanes = getLaneCountAtIntersection(turn_node, node_based_graph);

    for (const EdgeID onto_edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
    {
        BOOST_ASSERT(onto_edge != SPECIAL_EDGEID);
        const NodeID to_node = node_based_graph.GetTarget(onto_edge);
        const auto &onto_data = node_based_graph.GetEdgeData(onto_edge);

        bool turn_is_valid =
            // reverse edges are never valid turns because the resulting turn would look like this:
            // from_node --via_edge--> turn_node <--onto_edge-- to_node
            // however we need this for capture intersection shape for incoming one-ways
            !onto_data.reversed &&
            // we are not turning over a barrier
            (!is_barrier_node || from_node == to_node) &&
            // We are at an only_-restriction but not at the right turn.
            (only_restriction_to_node == SPECIAL_NODEID || to_node == only_restriction_to_node) &&
            // the turn is not restricted
            !restriction_map.CheckIfTurnIsRestricted(from_node, turn_node, to_node);

        auto angle = 0.;
        double bearing = 0.;

        // The first coordinate (the origin) can depend on the number of lanes turning onto,
        // just as the target coordinate can. Here we compute the corrected coordinate for the
        // incoming edge.
        const auto first_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
            from_node, via_eid, INVERT, turn_node, intersection_lanes);

        if (from_node == to_node)
        {
            bearing = util::coordinate_calculation::bearing(turn_coordinate, first_coordinate);
            uturn_could_be_valid = turn_is_valid;
            if (turn_is_valid && !is_barrier_node)
            {
                // we only add u-turns for dead-end streets.
                if (node_based_graph.GetOutDegree(turn_node) > 1)
                {
                    auto number_of_emmiting_bidirectional_edges = 0;
                    for (auto edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
                    {
                        auto target = node_based_graph.GetTarget(edge);
                        auto reverse_edge = node_based_graph.FindEdge(target, turn_node);
                        BOOST_ASSERT(reverse_edge != SPECIAL_EDGEID);
                        if (!node_based_graph.GetEdgeData(reverse_edge).reversed)
                        {
                            ++number_of_emmiting_bidirectional_edges;
                        }
                    }
                    // is a dead-end, only possible road is to go back
                    turn_is_valid = number_of_emmiting_bidirectional_edges <= 1;
                }
            }
            has_uturn_edge = true;
            BOOST_ASSERT(angle >= 0. && angle < std::numeric_limits<double>::epsilon());
        }
        else
        {
            // the default distance we lookahead on a road. This distance prevents small mapping
            // errors to impact the turn angles.
            const auto third_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
                turn_node, onto_edge, !INVERT, to_node, intersection_lanes);

            angle = util::coordinate_calculation::computeAngle(
                first_coordinate, turn_coordinate, third_coordinate);

            bearing = util::coordinate_calculation::bearing(turn_coordinate, third_coordinate);

            if (std::abs(angle) < std::numeric_limits<double>::epsilon())
                has_uturn_edge = true;
        }
        intersection.push_back(
            ConnectedRoad(TurnOperation{onto_edge,
                                        angle,
                                        bearing,
                                        {TurnType::Invalid, DirectionModifier::UTurn},
                                        INVALID_LANE_DATAID},
                          turn_is_valid));
    }

    // We hit the case of a street leading into nothing-ness. Since the code here assumes
    // that this
    // will never happen we add an artificial invalid uturn in this case.
    if (!has_uturn_edge)
    {
        const auto first_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
            from_node,
            via_eid,
            INVERT,
            turn_node,
            node_based_graph.GetEdgeData(via_eid).road_classification.GetNumberOfLanes());
        const double bearing =
            util::coordinate_calculation::bearing(turn_coordinate, first_coordinate);

        intersection.push_back({TurnOperation{via_eid,
                                              0.,
                                              bearing,
                                              {TurnType::Invalid, DirectionModifier::UTurn},
                                              INVALID_LANE_DATAID},
                                false});
    }

    std::sort(std::begin(intersection),
              std::end(intersection),
              std::mem_fn(&ConnectedRoad::compareByAngle));

    BOOST_ASSERT(intersection[0].angle >= 0. &&
                 intersection[0].angle < std::numeric_limits<double>::epsilon());

    const auto valid_count =
        boost::count_if(intersection, [](const ConnectedRoad &road) { return road.entry_allowed; });
    if (0 == valid_count && uturn_could_be_valid)
    {
        // after intersections sorting by angles, find the u-turn with (from_node ==
        // to_node)
        // that was inserted together with setting uturn_could_be_valid flag
        std::size_t self_u_turn = 0;
        while (self_u_turn < intersection.size() &&
               intersection[self_u_turn].angle < std::numeric_limits<double>::epsilon() &&
               from_node != node_based_graph.GetTarget(intersection[self_u_turn].eid))
        {
            ++self_u_turn;
        }

        BOOST_ASSERT(from_node == node_based_graph.GetTarget(intersection[self_u_turn].eid));
        intersection[self_u_turn].entry_allowed = true;
    }
    return intersection;
}

// Checks for mergability of two ways that represent the same intersection. For further information
// see interface documentation in header.
bool IntersectionGenerator::CanMerge(const NodeID node_at_intersection,
                                     const Intersection &intersection,
                                     std::size_t first_index,
                                     std::size_t second_index) const
{
    const auto &first_data = node_based_graph.GetEdgeData(intersection[first_index].eid);
    const auto &second_data = node_based_graph.GetEdgeData(intersection[second_index].eid);

    // don't merge on degree two, since it's most likely a bollard/traffic light or a round way
    if (intersection.size() <= 2)
        return false;

    // only merge named ids
    if (first_data.name_id == EMPTY_NAMEID || second_data.name_id == EMPTY_NAMEID)
        return false;

    // need to be same name
    if (util::guidance::requiresNameAnnounced(
            first_data.name_id, second_data.name_id, name_table, street_name_suffix_table))
        return false;

    if (!canMergeRoad(node_at_intersection,
                      intersection[first_index],
                      intersection[second_index],
                      node_based_graph,
                      *this,
                      node_info_list,
                      coordinate_extractor))
        return false;

    else
        return true;
#if 0
    // mergeable if the angle is not too big
    const auto angle_between =
        angularDeviation(intersection[first_index].angle, intersection[second_index].angle);

    const auto intersection_lanes = intersection.getHighestConnectedLaneCount(node_based_graph);

    const auto coordinate_at_in_edge =
        coordinate_extractor.GetCoordinateAlongRoad(node_at_intersection,
                                                    intersection[0].eid,
                                                    !INVERT,
                                                    node_based_graph.GetTarget(intersection[0].eid),
                                                    intersection_lanes);

    const auto coordinate_at_intersection = node_info_list[node_at_intersection];

    const auto isValidYArm = [this,
                              intersection,
                              coordinate_at_in_edge,
                              coordinate_at_intersection,
                              node_at_intersection](const std::size_t index,
                                                    const std::size_t other_index) {
        const auto GetActualTarget = [&](const std::size_t index) {
            EdgeID last_in_edge_id;
            GetActualNextIntersection(
                node_at_intersection, intersection[index].eid, nullptr, &last_in_edge_id);
            return node_based_graph.GetTarget(last_in_edge_id);
        };

        const auto target_id = GetActualTarget(index);
        const auto other_target_id = GetActualTarget(other_index);
        if (target_id == node_at_intersection || other_target_id == node_at_intersection)
            return false;

        const auto coordinate_at_target = node_info_list[target_id];
        const auto coordinate_at_other_target = node_info_list[other_target_id];

        const auto turn_angle = util::coordinate_calculation::computeAngle(
            coordinate_at_in_edge, coordinate_at_intersection, coordinate_at_target);
        const auto other_turn_angle = util::coordinate_calculation::computeAngle(
            coordinate_at_in_edge, coordinate_at_intersection, coordinate_at_other_target);

        const bool becomes_narrower =
            angularDeviation(turn_angle, other_turn_angle) < NARROW_TURN_ANGLE &&
            angularDeviation(turn_angle, other_turn_angle) <=
                angularDeviation(intersection[index].angle, intersection[other_index].angle);

        const bool has_same_deviation =
            std::abs(angularDeviation(intersection[index].angle, STRAIGHT_ANGLE) -
                     angularDeviation(intersection[other_index].angle, STRAIGHT_ANGLE)) <
            MAXIMAL_ALLOWED_NO_TURN_DEVIATION;

        return becomes_narrower || has_same_deviation;
    };

    const bool is_y_arm_first = isValidYArm(first_index, second_index);
    const bool is_y_arm_second = isValidYArm(second_index, first_index);

    // Only merge valid y-arms
    if (!is_y_arm_first || !is_y_arm_second)
        return false;

    if (angular_deviation < 60)
        return true;

    // Finally, we also allow merging if all streets offer the same name, it is only three roads and
    // the angle is not fully extreme:
    if (intersection.size() != 3)
        return false;

    // since we have an intersection of size three now, there is only one index we are not looking
    // at right now. The final index in the intersection is calculated next:
    const std::size_t third_index = [first_index, second_index]() {
        if (first_index == 0)
            return second_index == 2 ? 1 : 2;
        else if (first_index == 1)
            return second_index == 2 ? 0 : 2;
        else
            return second_index == 1 ? 0 : 1;
    }();

    // needs to be same road coming in
    const auto &third_data = node_based_graph.GetEdgeData(intersection[third_index].eid);

    if (third_data.name_id != EMPTY_NAMEID &&
        util::guidance::requiresNameAnnounced(
            third_data.name_id, first_data.name_id, name_table, street_name_suffix_table))
        return false;

    // we only allow collapsing of a Y like fork. So the angle to the third index has to be
    // roughly equal:
    const auto y_angle_difference = angularDeviation(
        angularDeviation(intersection[third_index].angle, intersection[first_index].angle),
        angularDeviation(intersection[third_index].angle, intersection[second_index].angle));

    // Allow larger angles if its three roads only of the same name
    // This is a heuristic and might need to be revised.
    const bool assume_y_intersection =
        angular_deviation < 100 && y_angle_difference < FUZZY_ANGLE_DIFFERENCE;
    return assume_y_intersection;
#endif
}

/*
 * Segregated Roads often merge onto a single intersection.
 * While technically representing different roads, they are
 * often looked at as a single road.
 * Due to the merging, turn Angles seem off, wenn we compute them from the
 * initial positions.
 *
 *         b<b<b<b(1)<b<b<b
 * aaaaa-b
 *         b>b>b>b(2)>b>b>b
 *
 * Would be seen as a slight turn going fro a to (2). A Sharp turn going from
 * (1) to (2).
 *
 * In cases like these, we megre this segregated roads into a single road to
 * end up with a case like:
 *
 * aaaaa-bbbbbb
 *
 * for the turn representation.
 * Anything containing the first u-turn in a merge affects all other angles
 * and is handled separately from all others.
 */
Intersection IntersectionGenerator::MergeSegregatedRoads(const NodeID intersection_node,
                                                         Intersection intersection) const
{
    // intersections with only a single road are not considered
    if (intersection.size() <= 1)
        return intersection;

    // TODO remove
    const auto intersection_copy = intersection;
    bool merged = false;

    const auto getRight = [&](std::size_t index) {
        return (index + intersection.size() - 1) % intersection.size();
    };

    const auto merge = [](const ConnectedRoad &first,
                          const ConnectedRoad &second) -> ConnectedRoad {
        ConnectedRoad result = first.entry_allowed ? first : second;
        result.angle = angleBetween(first.angle, second.angle);
        result.bearing = angleBetween(first.bearing, second.bearing);
        BOOST_ASSERT(0 <= result.angle && result.angle <= 360.0);
        BOOST_ASSERT(0 <= result.bearing && result.bearing <= 360.0);
        return result;
    };

    const bool is_connected_to_roundabout = [this, &intersection]() {
        for (const auto &road : intersection)
        {
            if (node_based_graph.GetEdgeData(road.eid).roundabout)
                return true;
        }
        return false;
    }();

    // check for merges including the basic u-turn
    // these result in an adjustment of all other angles. This is due to how these angles are
    // perceived. Considering the following example:
    //
    //   c   b
    //     Y
    //     a
    //
    // coming from a to b (given a road that splits at the fork into two one-ways), the turn is not
    // considered as a turn but rather as going straight.
    // Now if we look at the situation merging:
    //
    //  a     b
    //    \ /
    // e - + - d
    //     |
    //     c
    //
    // With a,b representing the same road, the intersection itself represents a classif for way
    // intersection so we handle it like
    //
    //   (a),b
    //      |
    // e -  + - d
    //      |
    //      c
    //
    // To be able to consider this adjusted representation down the line, we merge some roads.
    // If the merge occurs at the u-turn edge, we need to adjust all angles, though, since they are
    // with respect to the now changed perceived location of a. If we move (a) to the left, we add
    // the difference to all angles. Otherwise we subtract it.
    bool merged_first = false;
    // these result in an adjustment of all other angles
    if (CanMerge(intersection_node, intersection, 0, intersection.size() - 1))
    {
        merged = true;
        merged_first = true;
        // moving `a` to the left
        const double correction_factor = (360 - intersection[intersection.size() - 1].angle) / 2;
        for (std::size_t i = 1; i + 1 < intersection.size(); ++i)
            intersection[i].angle += correction_factor;

        // FIXME if we have a left-sided country, we need to switch this off and enable it
        // below
        intersection[0] = merge(intersection.front(), intersection.back());
        intersection[0].angle = 0;
        intersection.pop_back();
    }
    else if (CanMerge(intersection_node, intersection, 0, 1))
    {
        merged = true;
        merged_first = true;
        // moving `a` to the right
        const double correction_factor = (intersection[1].angle) / 2;
        for (std::size_t i = 2; i < intersection.size(); ++i)
            intersection[i].angle -= correction_factor;
        intersection[0] = merge(intersection[0], intersection[1]);
        intersection[0].angle = 0;
        intersection.erase(intersection.begin() + 1);
    }

    if (merged_first && is_connected_to_roundabout)
    {
        /*
         * We are merging a u-turn against the direction of a roundabout
         *
         *     -----------> roundabout
         *        /    \
         *     out      in
         *
         * These cases have to be disabled, even if they are not forbidden specifically by a
         * relation
         */
        intersection[0].entry_allowed = false;
    }


    const auto invalidate_road = [](ConnectedRoad &road) { road.eid = SPECIAL_EDGEID; };
    for (std::size_t index = 2; index < intersection.size(); ++index)
    {
        const auto previous_index = getRight(index);
        if (intersection[previous_index].eid != SPECIAL_EDGEID &&
            CanMerge(intersection_node, intersection, index, previous_index))
        {
            merged = true;
            intersection[previous_index] = merge(intersection[previous_index], intersection[index]);
            invalidate_road(intersection[index]);
        }
    }

    // remove all invalid turns
    intersection.erase(
        std::remove_if(intersection.begin(),
                       intersection.end(),
                       [](const ConnectedRoad &road) { return road.eid == SPECIAL_EDGEID; }),
        intersection.end());

    if (merged)
        util::ScopedGeojsonLoggerGuard<extractor::IntersectionPrinter>::Write(intersection_node,
                                                                              intersection_copy);

    std::sort(std::begin(intersection),
              std::end(intersection),
              std::mem_fn(&ConnectedRoad::compareByAngle));

    return intersection;
}

// OSM can have some very steep angles for joining roads. Considering the following intersection:
//        x
//        |
//        v __________c
//       /
// a ---d
//       \ __________b
//
// with c->d as a oneway
// and d->b as a oneway, the turn von x->d is actually a turn from x->a. So when looking at the
// intersection coming from x, we want to interpret the situation as
//           x
//           |
// a __ d __ v__________c
//      |
//      |_______________b
//
// Where we see the turn to `d` as a right turn, rather than going straight.
// We do this by adjusting the local turn angle at `x` to turn onto `d` to be reflective of this
// situation, where `v` would be the node at the intersection.
Intersection IntersectionGenerator::AdjustForJoiningRoads(const NodeID node_at_intersection,
                                                          Intersection intersection) const
{
    // nothing to do for dead ends
    if (intersection.size() <= 1)
        return intersection;

    const util::Coordinate coordinate_at_intersection = node_info_list[node_at_intersection];
    // never adjust u-turns
    for (std::size_t index = 1; index < intersection.size(); ++index)
    {
        auto &road = intersection[index];
        // to find out about the above situation, we need to look at the next intersection (at d in
        // the example). If the initial road can be merged to the left/right, we are about to adjust
        // the angle.
        const auto next_intersection_along_road = GetConnectedRoads(node_at_intersection, road.eid);

        if (next_intersection_along_road.size() <= 1)
            continue;

        const auto node_at_next_intersection = node_based_graph.GetTarget(road.eid);
        const util::Coordinate coordinate_at_next_intersection =
            node_info_list[node_at_next_intersection];
        if (util::coordinate_calculation::haversineDistance(coordinate_at_intersection,
                                                            coordinate_at_next_intersection) > 30)
            continue;

        const auto adjustAngle = [](double angle, double offset) {
            angle += offset;
            if (angle > 360)
                return angle - 360.;
            else if (angle < 0)
                return angle + 360.;
            return angle;
        };

        const auto range = node_based_graph.GetAdjacentEdgeRange(node_at_next_intersection);
        if (range.size() <= 1)
            continue;

        // the order does not matter
        const auto get_offset = [](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
            return 0.5 * angularDeviation(lhs.angle, rhs.angle);
        };

        // When offsetting angles in our turns, we don't want to get past the next turn. This
        // function simply limits an offset to be at most half the distance to the next turn in the
        // offfset direction
        const auto get_corrected_offset = [](const double offset,
                                             const ConnectedRoad &road,
                                             const ConnectedRoad &next_road_in_offset_direction) {
            const auto offset_limit =
                angularDeviation(road.angle, next_road_in_offset_direction.angle);
            // limit the offset with an additional buffer
            return (offset + MAXIMAL_ALLOWED_NO_TURN_DEVIATION > offset_limit) ? 0.5 * offset_limit
                                                                               : offset;
        };

        // check if the u-turn edge at the next intersection could be merged to the left/right. If
        // this is the case and the road is not far away (see previous distance check), if
        // influences the perceived angle.
        if (CanMerge(node_at_next_intersection, next_intersection_along_road, 0, 1))
        {
            const auto offset =
                get_offset(next_intersection_along_road[0], next_intersection_along_road[1]);

            const auto corrected_offset =
                get_corrected_offset(offset, road, intersection[(index + 1) % intersection.size()]);
            // at the target intersection, we merge to the right, so we need to shift the current
            // angle to the left
            road.angle = adjustAngle(road.angle, corrected_offset);
            road.bearing = adjustAngle(road.bearing, corrected_offset);
        }
        else if (CanMerge(node_at_next_intersection,
                          next_intersection_along_road,
                          0,
                          next_intersection_along_road.size() - 1))
        {
            const auto offset =
                get_offset(next_intersection_along_road[0],
                           next_intersection_along_road[next_intersection_along_road.size() - 1]);

            const auto corrected_offset =
                get_corrected_offset(offset, road, intersection[index - 1]);
            // at the target intersection, we merge to the left, so we need to shift the current
            // angle to the right
            road.angle = adjustAngle(road.angle, -corrected_offset);
            road.bearing = adjustAngle(road.bearing, -corrected_offset);
        }
    }
    return intersection;
}

Intersection
IntersectionGenerator::GetActualNextIntersection(const NodeID starting_node,
                                                 const EdgeID via_edge,
                                                 NodeID *resulting_from_node = nullptr,
                                                 EdgeID *resulting_via_edge = nullptr) const
{
    // This function skips over traffic lights/graph compression issues and similar to find the next
    // actual intersection
    Intersection result = GetConnectedRoads(starting_node, via_edge);

    // Skip over stuff that has not been compressed due to barriers/parallel edges
    NodeID node_at_intersection = starting_node;
    EdgeID incoming_edge = via_edge;

    // to prevent endless loops
    const auto termination_node = node_based_graph.GetTarget(via_edge);

    // using a maximum lookahead, we make sure not to end up in some form of loop
    std::unordered_set<NodeID> visited_nodes;
    while (visited_nodes.count(node_at_intersection) == 0 &&
           (result.size() == 2 &&
            node_based_graph.GetEdgeData(via_edge).IsCompatibleTo(
                node_based_graph.GetEdgeData(result[1].eid))))
    {
        visited_nodes.insert(node_at_intersection);
        node_at_intersection = node_based_graph.GetTarget(incoming_edge);
        incoming_edge = result[1].eid;
        result = GetConnectedRoads(node_at_intersection, incoming_edge);

        // When looping back to the original node, we obviously are in a loop. Stop there.
        if (termination_node == node_based_graph.GetTarget(incoming_edge))
            break;
    }

    // return output if requested
    if (resulting_from_node)
        *resulting_from_node = node_at_intersection;
    if (resulting_via_edge)
        *resulting_via_edge = incoming_edge;

    return result;
}

const CoordinateExtractor &IntersectionGenerator::GetCoordinateExtractor() const
{
    return coordinate_extractor;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
