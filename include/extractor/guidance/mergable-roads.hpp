#ifndef OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS
#define OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS

#include "extractor/guidance/coordinate_extractor.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/node_based_graph_walker.hpp"
#include "extractor/query_node.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Forward declaration to be able to access the intersection_generator
class IntersectionGenerator;

/*
 * When it comes to merging roads, we need to find out if two ways actually represent the same road.
 * This check tries to identify roads which are the same road in opposite directions
 */
inline bool haveCompatibleRoadData(const util::NodeBasedEdgeData &lhs_edge_data,
                                   const util::NodeBasedEdgeData &rhs_edge_data)
{
    // to describe the same road, but in opposite directions (which is what we require for a
    // merge), the roads have to feature one reversed and one non-reversed edge
    if (lhs_edge_data.reversed == rhs_edge_data.reversed)
        return false;

    // The travel mode should be the same for both roads. If we were to merge different travel
    // modes, we would hide information/run the risk of loosing valid choices (e.g. short period
    // of pushing)
    if (lhs_edge_data.travel_mode != rhs_edge_data.travel_mode)
        return false;

    return lhs_edge_data.road_classification == rhs_edge_data.road_classification;
}

inline auto makeCheckRoadForName(const NameID name_id,
                                 const util::NodeBasedDynamicGraph &node_based_graph)
{
    return [name_id, &node_based_graph](const ConnectedRoad &road) {
        return name_id == node_based_graph.GetEdgeData(road.eid).name_id;
    };
}

inline bool isNarrowTriangle(const NodeID intersection_node,
                             const ConnectedRoad &lhs,
                             const ConnectedRoad &rhs,
                             const util::NodeBasedDynamicGraph &node_based_graph,
                             const std::vector<QueryNode> &node_coordinates,
                             const IntersectionGenerator &intersection_generator)
{
    // selection data to the right and left
    IntersectionFinderAccumulator left_accumulator(5, intersection_generator),
        right_accumulator(5, intersection_generator);

    // Standard following the straightmost road
    // Since both items have the same id, we can `select` based on any setup
    SelectRoadByNameOnlyChoiceAndStraightness selector(
        node_based_graph.GetEdgeData(lhs.eid).name_id, false);

    NodeBasedGraphWalker graph_walker(node_based_graph, intersection_generator);
    graph_walker.TraverseRoad(intersection_node, lhs.eid, left_accumulator, selector);
    graph_walker.TraverseRoad(intersection_node, rhs.eid, right_accumulator, selector);

    BOOST_ASSERT(!left_accumulator.intersection.empty() && !right_accumulator.intersection.empty());

    // find the closes resembling a right turn
    const auto connector_turn = left_accumulator.intersection.findClosestTurn(90);
    // check if that right turn connects to the right_accumulator intersection (i.e. we have a
    // triangle)

    // a connection should be somewhat to the right
    if (angularDeviation(connector_turn->angle, 90) > NARROW_TURN_ANGLE)
        return false;

    // the width we can bridge at the intersection
    const auto assumed_lane_width =
        .5 * getLaneCountAtIntersection(intersection_node, node_based_graph) * 3.25;

    if (util::coordinate_calculation::haversineDistance(
            node_coordinates[node_based_graph.GetTarget(left_accumulator.via_edge_id)],
            node_coordinates[node_based_graph.GetTarget(right_accumulator.via_edge_id)]) >
        (2 * assumed_lane_width + 8))
        return false;

    // check if both intersections are connected
    IntersectionFinderAccumulator connect_accumulator(5, intersection_generator);
    graph_walker.TraverseRoad(node_based_graph.GetTarget(left_accumulator.via_edge_id),
                              connector_turn->eid,
                              connect_accumulator,
                              selector);

    // the if both items are connected
    return node_based_graph.GetTarget(connect_accumulator.via_edge_id) ==
           node_based_graph.GetTarget(right_accumulator.via_edge_id);
}

inline bool connectAgain(const NodeID intersection_node,
                         const ConnectedRoad &lhs,
                         const ConnectedRoad &rhs,
                         const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator)
{
    // compute the set of all intersection_nodes along the way of an edge, until it reaches a
    // location with the same name repeatet at least three times
    const auto findMeetUpCandidate = [&](const NameID searched_name, const ConnectedRoad &road) {
        auto current_node = intersection_node;
        auto current_eid = road.eid;

        const auto has_requested_name = makeCheckRoadForName(searched_name, node_based_graph);
        // limit our search to at most 10 intersections. This is intended to ignore connections that
        // are really far away
        for (std::size_t hop_count = 0; hop_count < 10; ++hop_count)
        {
            const auto next_intersection =
                intersection_generator.GetConnectedRoads(current_node, current_eid);
            const auto count = std::count_if(
                next_intersection.begin() + 1, next_intersection.end(), has_requested_name);

            if (count >= 2)
                return node_based_graph.GetTarget(current_eid);
            else if (count == 0)
            {
                return SPECIAL_NODEID;
            }
            else
            {
                current_node = node_based_graph.GetTarget(current_eid);
                // skip over bridges/similar
                if (next_intersection.size() == 2)
                    current_eid = next_intersection[1].eid;
                else
                {
                    const auto next_turn = std::find_if(
                        next_intersection.begin() + 1, next_intersection.end(), has_requested_name);

                    if (angularDeviation(next_turn->angle, 180) > NARROW_TURN_ANGLE)
                        return current_node;
                    BOOST_ASSERT(next_turn != next_intersection.end());
                    current_eid = next_turn->eid;
                }
            }
        }

        return SPECIAL_NODEID;
    };

    const auto left_candidate =
        findMeetUpCandidate(node_based_graph.GetEdgeData(lhs.eid).name_id, lhs);
    const auto right_candidate =
        findMeetUpCandidate(node_based_graph.GetEdgeData(rhs.eid).name_id, rhs);

    return left_candidate == right_candidate && left_candidate != SPECIAL_NODEID &&
           left_candidate != intersection_node;
}

// Check if two roads go in the general same direction
inline bool haveSameDirection(const NodeID intersection_node,
                              const ConnectedRoad &lhs,
                              const ConnectedRoad &rhs,
                              const util::NodeBasedDynamicGraph &node_based_graph,
                              const IntersectionGenerator &intersection_generator,
                              const std::vector<QueryNode> &node_coordinates,
                              const CoordinateExtractor &coordinate_extractor)
{
    if (angularDeviation(lhs.angle, rhs.angle) > 90)
        return false;

    // Find a coordinate following a road that is far away
    NodeBasedGraphWalker graph_walker(node_based_graph, intersection_generator);
    const auto getCoordinatesAlongWay = [&](const EdgeID edge_id, const double max_length) {
        LengthLimitedCoordinateAccumulator accumulator(
            coordinate_extractor, node_based_graph, max_length);
        SelectRoadByNameOnlyChoiceAndStraightness selector(
            node_based_graph.GetEdgeData(edge_id).name_id, false);
        graph_walker.TraverseRoad(intersection_node, edge_id, accumulator, selector);
        return accumulator.coordinates;
    };

    const auto coordinates_to_the_left =
        coordinate_extractor.SampleCoordinates(getCoordinatesAlongWay(lhs.eid, 100), 100, 5);
    const auto coordinates_to_the_right =
        coordinate_extractor.SampleCoordinates(getCoordinatesAlongWay(rhs.eid, 100), 100, 5);

    // if we didn't traverse at least 35 meters (7 * 5 + first coordinate), we didn't go far enough
    if (coordinates_to_the_left.size() < 8 || coordinates_to_the_right.size() < 8)
        return false;

    // we allow some basic deviation for all roads. If the there are more lanes present, we allow
    // for a bit more deviation
    const auto max_deviation = [&]() {
        const auto num_lanes = [&node_based_graph](const ConnectedRoad &road) {
            return node_based_graph.GetEdgeData(road.eid)
                .road_classification.GetNumberOfLanes();
        };

        const auto lane_count = std::max<std::uint8_t>(2, std::max(num_lanes(lhs), num_lanes(rhs)));
        return 4 * sqrt(lane_count);
    }();

    const auto are_parallel = util::coordinate_calculation::areParallel(
        coordinates_to_the_left, coordinates_to_the_right, max_deviation);

    return are_parallel;

#if 0
    const auto coordinate_to_left = findCoordinateFollowingRoad(lhs, 5 + 4 * assumed_lane_width);
    const auto coordinate_to_right = findCoordinateFollowingRoad(rhs, 5 + 4 * assumed_lane_width);
    const auto angle = util::coordinate_calculation::computeAngle(
        coordinate_to_left, node_coordinates[intersection_node], coordinate_to_right);

    return std::min(angle, 360 - angle) < 20;
#endif
}

// Try if two roads can be merged into a single one, since they represent the same road
inline bool canMergeRoad(const NodeID intersection_node,
                         const ConnectedRoad &lhs,
                         const ConnectedRoad &rhs,
                         const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator,
                         const std::vector<QueryNode> &node_coordinates,
                         const CoordinateExtractor &coordinate_extractor)
{
    const auto &lhs_edge_data = node_based_graph.GetEdgeData(lhs.eid);
    const auto &rhs_edge_data = node_based_graph.GetEdgeData(rhs.eid);

    // roundabouts are special, simply don't hurt them. We might not want to bear the consequences
    if (lhs_edge_data.roundabout || rhs_edge_data.roundabout)
        return false;

    // mergable roads cannot hide a turn. We are not allowed to remove any of them
    if (lhs.entry_allowed && rhs.entry_allowed)
        return false;

    // and they need to describe the same road
    if (!haveCompatibleRoadData(lhs_edge_data, rhs_edge_data))
        return false;

    if (angularDeviation(lhs.angle, rhs.angle) > 60)
        return false;

    if (false && isNarrowTriangle(intersection_node,
                                  lhs,
                                  rhs,
                                  node_based_graph,
                                  node_coordinates,
                                  intersection_generator))
        return true;

    if (haveSameDirection(intersection_node,
                          lhs,
                          rhs,
                          node_based_graph,
                          intersection_generator,
                          node_coordinates,
                          coordinate_extractor))
        return true;

    // if (connectAgain(intersection_node, lhs, rhs, node_based_graph, intersection_generator))

    return false;
    // finally check if two roads describe the same way
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

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif
