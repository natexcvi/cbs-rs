use super::{
    lower_level::{find_shortest_path, Grid, LocationTime},
    search::AStarNode,
};
use std::collections::HashMap;

struct VertexConflict<'a> {
    agent1: &'a Agent,
    agent2: &'a Agent,
    time: i32,
    location: (i32, i32),
}

struct EdgeConflict<'a> {
    agent1: &'a Agent,
    agent2: &'a Agent,
    time: i32,
    location1: (i32, i32),
    location2: (i32, i32),
}

pub enum Conflict {
    VertexConflict,
    EdgeConflict,
}

pub struct Constraint<'a> {
    agent: &'a Agent,
    time: i32,
    location: (i32, i32),
}

pub type Path = Vec<(i32, i32)>;

#[derive(PartialEq, Eq, Hash, Clone)]
pub struct Agent {
    id: String,
    start: (i32, i32),
    goal: (i32, i32),
}

pub struct ConflictTreeNode<'a> {
    constraints: Vec<&'a Constraint<'a>>,
    paths: HashMap<&'a Agent, Path>,
}

impl<'a> ConflictTreeNode<'a> {
    fn new() -> ConflictTreeNode<'a> {
        ConflictTreeNode {
            constraints: Vec::new(),
            paths: HashMap::new(),
        }
    }

    fn conflicts(&self) -> Vec<&Conflict> {
        // TODO: implement
        Vec::new()
    }

    fn calculate_paths(&mut self) {
        for agent in self.paths.keys() {
            let path = find_shortest_path(
                Grid {
                    width: 10,
                    height: 10,
                    obstacles: self.constraints.iter().filter(|c| c.agent == *agent).map(|c| LocationTime { location: c.location, time: c.time }).collect(),
                    goal: agent.goal,
                },
                LocationTime {
                    location: agent.start,
                    time: 0,
                },
            );
            self.paths.insert(agent, path.unwrap().iter().map(|n| n.location).collect());
        }
    }
}

impl AStarNode for ConflictTreeNode<'_> {
    fn g(&self) -> f64 {
        self.constraints.len() as f64
    }

    fn h(&self) -> f64 {
        self.conflicts().len() as f64
    }

    fn expand(&self) -> Vec<Box<Self>> {
        let mut expanded = Vec::<Box<Self>>::new();
        expanded
    }
}
