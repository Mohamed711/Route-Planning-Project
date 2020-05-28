#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Specify the start, and end nodes based on the given coordinates
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// Heuristic value is based on the distance of the node from the end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// Add the neighbour nodes to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node* node : current_node->neighbors)
    {
        if (node->visited == false)
        {
          	node->parent = current_node;
          	node->h_value = CalculateHValue(node);
          	node->g_value = current_node->g_value + current_node->distance(*node);
          	node->visited = true;
          	open_list.push_back(node);
        }
    }
}

// Return the node with the least cost (distance + heuristic for remianing distance)
RouteModel::Node* RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b)
                                             {return a->g_value + a->h_value > b->g_value + b->h_value;});
    RouteModel::Node * next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// Construct the path from the start node to the end node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // The distance is the g_value of the end node
    distance = current_node->g_value;

    // Follow the parent path till the start node
    while (current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

  	path_found.push_back(*start_node);

    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Start Searching for a path from the start to the end
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    // Mark the start node and add its neighbours
    current_node->visited = true;
    AddNeighbors(current_node);

    while (open_list.size() > 0)
    {
        current_node = NextNode();

        if (current_node == end_node)
        {
           m_Model.path = ConstructFinalPath(end_node);
           break;
        }

        AddNeighbors(current_node);
    }
}