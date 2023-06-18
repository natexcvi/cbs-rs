use std::collections::HashMap;

use super::{high_level::Agent, low_level::Grid, search::bfs};

struct MDDNode<T> {
    location: T,
    goal_reachable: bool,
    visited: bool,
    level: i32,
}

#[derive(Debug)]
pub(crate) enum MDDError {
    GoalUnreachable,
}

pub(crate) fn mdd(
    agent: &Agent,
    scenario: &Grid,
    c: i32,
) -> Result<Vec<Vec<(i32, i32)>>, MDDError> {
    let mut mdd = Vec::<Vec<(i32, i32)>>::new();
    for _ in 0..c + 1 {
        mdd.push(Vec::<(i32, i32)>::new());
    }
    let mut nodes = HashMap::<(i32, i32), MDDNode<(i32, i32)>>::new();
    let goal_node = MDDNode {
        location: agent.goal.clone(),
        goal_reachable: true,
        visited: false,
        level: 0,
    };
    nodes.insert(agent.goal.clone(), goal_node);
    bfs(
        &mut nodes,
        agent.goal.clone(),
        c,
        |nodes, node, level| {
            let cur_node = nodes.get_mut(&node).unwrap();

            cur_node.goal_reachable = true;
            cur_node.level = level;
            false
        },
        |nodes, node_location| {
            let node = nodes.get_mut(&node_location).unwrap();
            let mut neighbours = Vec::<(i32, i32)>::new();
            let mut neighbouring_cells = vec![
                (node.location.0 - 1, node.location.1),
                (node.location.0 + 1, node.location.1),
                (node.location.0, node.location.1 - 1),
                (node.location.0, node.location.1 + 1),
            ];
            neighbouring_cells.retain(|cell| scenario.is_valid_location(cell, &node_location));
            for neighbour in neighbouring_cells {
                if nodes.contains_key(&neighbour) {
                    continue;
                }
                let neighbour_node = MDDNode {
                    location: neighbour.clone(),
                    goal_reachable: false,
                    visited: false,
                    level: 0,
                };

                nodes.insert(neighbour.clone(), neighbour_node);
                neighbours.push(neighbour);
            }
            neighbours
        },
    );
    if !nodes[&agent.start].goal_reachable {
        return Err(MDDError::GoalUnreachable);
    }
    bfs(
        &mut nodes,
        agent.start.clone(),
        c,
        |nodes, node, level| {
            let cur_node = nodes.get_mut(&node).unwrap();
            if cur_node.visited {
                return true;
            }
            cur_node.visited = true;
            if cur_node.goal_reachable && cur_node.level + level <= c {
                mdd[level as usize].push(node);
            }
            false
        },
        |nodes, node_location| {
            let node = nodes.get_mut(&node_location).unwrap();
            let mut neighbours = Vec::<(i32, i32)>::new();
            let mut neighbouring_cells = vec![
                (node.location.0 - 1, node.location.1),
                (node.location.0 + 1, node.location.1),
                (node.location.0, node.location.1 - 1),
                (node.location.0, node.location.1 + 1),
            ];
            neighbouring_cells.retain(|cell| scenario.is_valid_location(cell, &node_location));
            for neighbour in neighbouring_cells {
                if !nodes.contains_key(&neighbour) || nodes[&neighbour].visited {
                    continue;
                }
                neighbours.push(neighbour);
            }
            neighbours
        },
    );

    Ok(mdd)
}

#[cfg(test)]
mod tests;
