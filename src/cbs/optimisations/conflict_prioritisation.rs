use std::collections::{HashMap, HashSet, VecDeque};
use std::hash::Hash;
use std::rc::Rc;

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
    G: FnMut(&mut HashMap<T, S>, T, i32),
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
                visit(nodes, node.clone(), cur_depth);
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

fn mdd(agent: &Agent, scenario: &Grid, c: i32) -> Vec<Vec<(i32, i32)>> {
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
            neighbouring_cells.retain(|cell| scenario.is_valid_location(cell));
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
    bfs(
        &mut nodes,
        agent.start.clone(),
        c,
        |nodes, node, level| {
            let cur_node = nodes.get_mut(&node).unwrap();
            if cur_node.visited {
                return;
            }
            cur_node.visited = true;
            if cur_node.goal_reachable && cur_node.level + level <= c {
                mdd[level as usize].push(node);
            }
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
            neighbouring_cells.retain(|cell| scenario.is_valid_location(cell));
            for neighbour in neighbouring_cells {
                if !nodes.contains_key(&neighbour) || nodes[&neighbour].visited {
                    continue;
                }
                neighbours.push(neighbour);
            }
            neighbours
        },
    );

    mdd
}

pub fn pick_conflict<'a>(conflicts: &Vec<Box<Conflict<'a>>>) -> Option<&'a Box<Conflict<'a>>> {
    let mut min_conflict = None;
    for conflict in conflicts {}
    min_conflict
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    #[rstest]
    fn test_mdd() {
        let scenario = crate::cbs::low_level::Grid::new(10, 10, vec![], (0, 0));
        let agent = crate::cbs::high_level::Agent {
            id: "a".to_string(),
            start: (0, 0),
            goal: (5, 5),
        };
        let mdd = super::mdd(&agent, &scenario, 10);
        assert_eq!(mdd.len(), 11);
        assert_eq!(mdd[0].len(), 1);
        assert_eq!(mdd[1].len(), 2);
        assert_eq!(mdd[2].len(), 3);
        assert_eq!(mdd[3].len(), 4);
        assert_eq!(mdd[4].len(), 5);
        assert_eq!(mdd[5].len(), 6);
        assert_eq!(mdd[6].len(), 5);
        assert_eq!(mdd[7].len(), 4);
        assert_eq!(mdd[8].len(), 3);
        assert_eq!(mdd[9].len(), 2);
        assert_eq!(mdd[10].len(), 1);
    }
}
