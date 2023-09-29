use std::collections::HashMap;

use cached::proc_macro::cached;
use cached::{SizedCache, UnboundCache};

use super::{
    high_level::Agent,
    low_level::{Grid, LocationTime},
    search::bfs,
};

struct MDDNode<T> {
    location: T,
    goal_reachable: bool,
    last_visited_level: i32,
    level: i32,
}

#[derive(Debug, Clone)]
pub(crate) enum MDDError {
    GoalUnreachable,
}

#[cached(
    type = "SizedCache<String, Result<Vec<Vec<(i32, i32)>>, MDDError>>",
    create = "{ SizedCache::with_size(10000) }",
    convert = r#"{ format!("{:?}{:?}{:?}", agent, scenario, c) }"#
)]
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
        last_visited_level: -1,
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
        |nodes, node_location, level| {
            let node = nodes.get_mut(&node_location).unwrap();
            let mut neighbours = Vec::<(i32, i32)>::new();
            let mut neighbouring_cells = vec![
                (node.location.0 - 1, node.location.1),
                (node.location.0 + 1, node.location.1),
                (node.location.0, node.location.1 - 1),
                (node.location.0, node.location.1 + 1),
            ];
            neighbouring_cells.retain(|cell| {
                // scenario.is_valid_location_time(
                //     &LocationTime::new(*cell, c - (level + 1)),
                //     &node_location,
                // )
                scenario.is_valid_location(cell, &node_location)
            });
            for neighbour in neighbouring_cells {
                if nodes.contains_key(&neighbour) {
                    continue;
                }
                let neighbour_node = MDDNode {
                    location: neighbour.clone(),
                    goal_reachable: false,
                    last_visited_level: -1,
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
            if cur_node.last_visited_level == level {
                return true;
            }
            cur_node.last_visited_level = level;
            if cur_node.goal_reachable && cur_node.level + level <= c {
                mdd[level as usize].push(node);
            }
            false
        },
        |nodes, node_location, level| {
            let node = nodes.get_mut(&node_location).unwrap();
            let mut neighbours = Vec::<(i32, i32)>::new();
            let mut neighbouring_cells = vec![
                (node.location.0 - 1, node.location.1),
                (node.location.0 + 1, node.location.1),
                (node.location.0, node.location.1 - 1),
                (node.location.0, node.location.1 + 1),
                (node.location.0, node.location.1),
            ];
            neighbouring_cells.retain(|cell| {
                scenario
                    .is_valid_location_time(&LocationTime::new(*cell, level + 1), &node_location)
            });
            for neighbour in neighbouring_cells {
                if !nodes.contains_key(&neighbour) {
                    continue;
                }
                neighbours.push(neighbour);
            }
            neighbours
        },
    );
    Ok(mdd)
}

#[cached(
    type = "SizedCache<String, Vec<Vec<((i32, i32), (i32, i32))>>>",
    create = "{ SizedCache::with_size(10000) }",
    convert = r#"{ format!("{:?}{:?}{:?}", mdd1, mdd2, c) }"#
)]
pub(crate) fn merge_mdds(
    mdd1: &Vec<Vec<(i32, i32)>>,
    mdd2: &Vec<Vec<(i32, i32)>>,
    c: i32,
) -> Vec<Vec<((i32, i32), (i32, i32))>> {
    let mut mdd = Vec::<Vec<((i32, i32), (i32, i32))>>::new();
    for _ in 0..c + 1 {
        mdd.push(Vec::<((i32, i32), (i32, i32))>::new());
    }
    for level in 0..c + 1 {
        let mdd1_level = mdd1
            .get(level as usize)
            .or_else(|| mdd1.last())
            .expect("mdd1 should not be empty");
        let mdd2_level = mdd2
            .get(level as usize)
            .or_else(|| mdd2.last())
            .expect("mdd2 should not be empty");
        for node1 in mdd1_level {
            for node2 in mdd2_level {
                let joint_loc = (node1.clone(), node2.clone());
                if node1 != node2
                    && !mdd[(level - 1).max(0) as usize].contains(&(joint_loc.1, joint_loc.0))
                {
                    mdd[level as usize].push(joint_loc);
                }
            }
        }
    }
    mdd
}

#[cfg(test)]
mod tests;
