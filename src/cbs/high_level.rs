use super::{
    low_level::{find_shortest_path, Grid, LocationTime},
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

pub enum Conflict<'a> {
    Vertex(VertexConflict<'a>),
    Edge(EdgeConflict<'a>),
}

#[derive(Clone)]
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
    constraints: Vec<Box<Constraint<'a>>>,
    agents: Vec<&'a Agent>,
    paths: HashMap<&'a Agent, Path>,
}

impl<'a> ConflictTreeNode<'a> {
    fn new(
        agents: Vec<&'a Agent>,
        constraints: Vec<Box<Constraint<'a>>>,
        precomputed_paths: HashMap<&'a Agent, Vec<(i32, i32)>>,
    ) -> ConflictTreeNode<'a> {
        let mut ctn = ConflictTreeNode {
            constraints,
            agents,
            paths: precomputed_paths,
        };
        ctn.calculate_paths();
        ctn
    }

    fn conflicts(&self) -> Vec<Box<Conflict>> {
        let mut conflicts = Vec::<Box<Conflict>>::new();
        let mut agent_locations = HashMap::<(i32, i32), Vec<&Agent>>::new();
        for i in 0..self.paths.values().map(|p| p.len()).max().unwrap() {
            for agent in self.agents.iter() {
                if i >= self.paths[agent].len() {
                    continue;
                }
                let location = self.paths[agent][i];
                agent_locations
                    .entry(location)
                    .or_insert(Vec::<&Agent>::new())
                    .push(agent);
                if agent_locations[&location].len() > 1 {
                    for agent2 in agent_locations[&location].iter() {
                        if agent2 == agent {
                            continue;
                        }
                        if i == 0 {
                            // TODO: fix condition
                            conflicts.push(Box::new(Conflict::Vertex(VertexConflict {
                                agent1: agent,
                                agent2: agent2,
                                time: i as i32,
                                location,
                            })));
                        } else {
                            conflicts.push(Box::new(Conflict::Edge(EdgeConflict {
                                agent1: agent,
                                agent2: agent2,
                                time: i as i32,
                                location1: self.paths[agent][i - 1],
                                location2: location,
                            })));
                        }
                    }
                }
                if i > 0 {
                    agent_locations
                        .entry(self.paths[agent][i - 1])
                        .or_default()
                        .retain(|a| a != agent);
                }
            }
        }
        conflicts
    }

    fn calculate_paths(&mut self) {
        for agent in self.agents.iter() {
            let path = find_shortest_path(
                Grid {
                    width: 10,
                    height: 10,
                    obstacles: self
                        .constraints
                        .iter()
                        .filter(|c| c.agent == *agent)
                        .map(|c| LocationTime {
                            location: c.location,
                            time: c.time,
                        })
                        .collect(),
                    goal: agent.goal,
                },
                LocationTime {
                    location: agent.start,
                    time: 0,
                },
            );
            self.paths
                .insert(agent, path.unwrap().iter().map(|n| n.location).collect());
        }
    }
}

impl<'a> AStarNode<'a> for ConflictTreeNode<'a> {
    fn g(&'a self) -> f64 {
        self.constraints.len() as f64
    }

    fn h(&'a self) -> f64 {
        self.conflicts().len() as f64
    }

    fn expand(&'a self) -> Vec<Box<Self>> {
        let mut expanded = Vec::<Box<Self>>::new();
        let conflicts = self.conflicts();
        if conflicts.is_empty() {
            return expanded;
        }
        let conflict = &*conflicts[0];
        match conflict {
            Conflict::Vertex(vc) => {
                for agent in vec![vc.agent1, vc.agent2] {
                    let constraint = Constraint {
                        agent,
                        time: vc.time,
                        location: vc.location,
                    };
                    let mut new_constraints = self.constraints.clone();
                    new_constraints.push(Box::new(constraint));
                    let mut new_paths = self.paths.clone();
                    new_paths.remove(agent);
                    expanded.push(Box::new(ConflictTreeNode::new(
                        self.agents.clone(),
                        new_constraints,
                        new_paths,
                    )));
                }
            }
            Conflict::Edge(ec) => {
                for agent in vec![ec.agent1, ec.agent2] {
                    let constraint1 = Constraint {
                        agent,
                        time: ec.time,
                        location: ec.location1,
                    };
                    let constraint2 = Constraint {
                        agent,
                        time: ec.time + 1,
                        location: ec.location2,
                    };
                    let mut new_constraints = self.constraints.clone();
                    new_constraints.push(Box::new(constraint1));
                    new_constraints.push(Box::new(constraint2));
                    expanded.push(Box::new(ConflictTreeNode::new(
                        self.agents.clone(),
                        new_constraints,
                        self.paths.clone(),
                    )));
                }
            }
        }
        expanded
    }
}
