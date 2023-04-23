use super::search::AStarNode;
use std::collections::HashMap;

struct VertexConflict {
    agent1: i32,
    agent2: i32,
    time: i32,
    location: (i32, i32),
}

struct EdgeConflict {
    agent1: i32,
    agent2: i32,
    time: i32,
    location1: (i32, i32),
    location2: (i32, i32),
}

pub enum Conflict {
    VertexConflict,
    EdgeConflict,
}

pub struct Constraint {
    agent: i32,
    time: i32,
    location: (i32, i32),
}

pub type Path = Vec<(i32, i32)>;

pub struct Agent {
    id: String,
    start: (i32, i32),
    goal: (i32, i32),
}

pub struct ConflictTreeNode<'a> {
    constraints: Vec<&'a Constraint>,
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
        // calculate paths for all agents given the
        // constraints, using AStar


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
        Vec::new()
    }
}
