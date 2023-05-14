use std::collections::{HashMap, HashSet, VecDeque};
use std::hash::Hash;
use std::rc::Rc;

use crate::cbs::high_level::Path;
use crate::cbs::{
    high_level::{Agent, Conflict},
    low_level::{Grid, LocationTime},
};

struct MDDNode<T> {
    location: T,
    goal_reachable: bool,
    visited: bool,
    level: i32,
}

enum BFSNode<T> {
    Node(T),
    Divider,
}

fn bfs<T, S, F, G>(nodes: &mut HashMap<T, S>, root: T, depth: i32, mut visit: G, mut expand: F)
where
    F: FnMut(&mut HashMap<T, S>, T) -> Vec<T>,
    G: FnMut(&mut HashMap<T, S>, T, i32) -> bool,
    T: Clone,
{
    let mut cur_depth = 0;
    let mut bfs_queue = VecDeque::<BFSNode<T>>::new();
    bfs_queue.push_back(BFSNode::Node(root));
    bfs_queue.push_back(BFSNode::Divider);
    while !bfs_queue.is_empty() {
        let node = bfs_queue.pop_front().unwrap();
        match node {
            BFSNode::Node(node) => {
                if visit(nodes, node.clone(), cur_depth) {
                    continue;
                }
                for neighbour in expand(nodes, node) {
                    bfs_queue.push_back(BFSNode::Node(neighbour));
                }
            }
            BFSNode::Divider => {
                cur_depth += 1;
                if cur_depth > depth {
                    break;
                }
                bfs_queue.push_back(BFSNode::Divider);
            }
        }
    }
}

#[derive(Debug)]
enum MDDError {
    GoalUnreachable,
}

fn mdd(agent: &Agent, scenario: &Grid, c: i32) -> Result<Vec<Vec<(i32, i32)>>, MDDError> {
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

#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum ConflictCardinality {
    Cardinal,
    SemiCardinal,
    NonCardinal,
}

fn cardinality(
    scenario: &Grid,
    paths: &HashMap<&Agent, Path>,
    conflict: &Conflict,
) -> ConflictCardinality {
    let agent1: &Agent;
    let agent2: &Agent;
    let time: i32;
    match conflict {
        Conflict::Vertex(c) => {
            agent1 = c.agent1;
            agent2 = c.agent2;
            time = c.time;
        }
        Conflict::Edge(c) => {
            agent1 = c.agent1;
            agent2 = c.agent2;
            time = c.time;
        }
    }
    let c1 = paths[agent1].len();
    let c2 = paths[agent2].len();
    let agent1_mdd = mdd(agent1, scenario, c1 as i32).unwrap();
    let agent2_mdd = mdd(agent2, scenario, c2 as i32).unwrap();
    if agent1_mdd[time.clone() as usize].len() == 1
        && agent2_mdd
            .get(time.clone() as usize)
            .unwrap_or(agent2_mdd.last().unwrap())
            .len()
            == 1
    {
        return ConflictCardinality::Cardinal;
    } else if agent1_mdd[time.clone() as usize].len() == 1
        || agent2_mdd
            .get(time.clone() as usize)
            .unwrap_or(agent2_mdd.last().unwrap())
            .len()
            == 1
    {
        return ConflictCardinality::SemiCardinal;
    } else {
        return ConflictCardinality::NonCardinal;
    }
}

pub fn pick_conflict<'a>(
    scenario: &Grid,
    paths: &HashMap<&Agent, Path>,
    conflicts: &Vec<Box<Conflict<'a>>>,
) -> Option<Box<Conflict<'a>>> {
    let min_conflict = conflicts
        .iter()
        .min_by(|a, b| cardinality(scenario, paths, a).cmp(&cardinality(scenario, paths, b)));
    match min_conflict {
        Some(c) => Some(c.clone()),
        None => None,
    }
}

#[cfg(test)]
mod tests;
