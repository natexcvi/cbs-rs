use super::{
    low_level::{find_shortest_path, Grid, LocationTime},
    search::{a_star, AStarNode},
};
use std::{collections::HashMap, error::Error, hash::Hash};

#[derive(Clone)]
pub struct VertexConflict<'a> {
    agent1: &'a Agent,
    agent2: &'a Agent,
    time: i32,
    location: (i32, i32),
}

#[derive(Clone)]
pub struct EdgeConflict<'a> {
    agent1: &'a Agent,
    agent2: &'a Agent,
    time: i32,
    location1: (i32, i32),
    location2: (i32, i32),
}

#[derive(Clone)]
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

#[derive(Clone)]
pub struct ConflictTreeNode<'a> {
    constraints: Vec<Box<Constraint<'a>>>,
    agents: Vec<&'a Agent>,
    paths: HashMap<&'a Agent, Path>,
    conflicts: Vec<Box<Conflict<'a>>>,
    scenario: &'a Grid,
}

impl<'a> ConflictTreeNode<'a> {
    fn new(
        agents: Vec<&'a Agent>,
        constraints: Vec<Box<Constraint<'a>>>,
        precomputed_paths: HashMap<&'a Agent, Vec<(i32, i32)>>,
        scenario: &'a Grid,
    ) -> ConflictTreeNode<'a> {
        let mut ctn = ConflictTreeNode {
            constraints,
            agents,
            paths: precomputed_paths,
            conflicts: Vec::<Box<Conflict>>::new(),
            scenario,
        };
        ctn.compute_paths();
        ctn.compute_conflicts();
        ctn
    }

    fn compute_conflicts(&mut self) {
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
                if i > 0 {
                    agent_locations
                        .entry(self.paths[agent][i - 1])
                        .or_default()
                        .retain(|a| a != agent);
                }
            }
            for (location, agents) in agent_locations.iter() {
                if agents.len() > 1 {
                    for (j, agent) in agents.iter().enumerate() {
                        for agent2 in agents[..j].iter() {
                            if i > 0
                                && i < self.paths[agent2].len()
                                && self.paths[agent2][i] == self.paths[agent][i - 1]
                            {
                                // TODO: fix condition
                                conflicts.push(Box::new(Conflict::Edge(EdgeConflict {
                                    agent1: agent,
                                    agent2: agent2,
                                    time: i as i32,
                                    location1: self.paths[agent][i - 1],
                                    location2: location.clone(),
                                })));
                            } else {
                                conflicts.push(Box::new(Conflict::Vertex(VertexConflict {
                                    agent1: agent,
                                    agent2: agent2,
                                    time: i as i32,
                                    location: location.clone(),
                                })));
                            }
                        }
                    }
                }
            }
        }
        self.conflicts.append(&mut conflicts);
    }

    fn compute_paths(&mut self) {
        for agent in self.agents.iter() {
            if self.paths.contains_key(agent) {
                continue;
            }
            let mut obstacles: Vec<LocationTime> = self
                .constraints
                .iter()
                .filter(|c| c.agent == *agent)
                .map(|c| LocationTime {
                    location: c.location,
                    time: c.time,
                })
                .collect();
            obstacles.append(&mut self.scenario.obstacles.clone());
            let path = find_shortest_path(
                Grid {
                    width: self.scenario.width,
                    height: self.scenario.height,
                    obstacles,
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

impl AStarNode<'_> for ConflictTreeNode<'_> {
    fn g(&self) -> f64 {
        self.paths.values().map(|p| p.len() as f64).sum()
    }

    fn h(&self) -> f64 {
        0.0 // TODO: more useful heuristic
    }

    fn is_goal(&self) -> bool {
        self.conflicts.is_empty()
    }

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        let mut expanded = Vec::<Box<Self>>::new();
        if self.conflicts.is_empty() {
            return Some(expanded);
        }
        let conflict = &*self.conflicts[0]; // TODO: pick conflict in a smarter way
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
                        self.scenario,
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
                        self.scenario,
                    )));
                }
            }
        }
        Some(expanded)
    }
}

pub struct CBS<'a> {
    scenario: &'a Grid,
    agents: Vec<&'a Agent>,
    high_level_expanded: usize,
    low_level_expanded: usize,
}

impl<'a> CBS<'a> {
    pub fn new(scenario: &'a Grid, agents: Vec<&'a Agent>) -> Self {
        CBS {
            scenario,
            agents,
            high_level_expanded: 0,
            low_level_expanded: 0,
        }
    }

    pub fn solve(&mut self) -> Result<HashMap<&Agent, Path>, Box<dyn Error>> {
        let root = ConflictTreeNode::new(
            self.agents.clone(),
            Vec::<Box<Constraint>>::new(),
            HashMap::<&Agent, Vec<(i32, i32)>>::new(),
            self.scenario,
        );
        let solution = a_star(root);
        match solution {
            Ok(solution) => {
                self.high_level_expanded += solution.nodes_expanded as usize;
                let last_node = solution.path.last().unwrap();
                let mut paths = HashMap::<&Agent, Path>::new();
                for agent in self.agents.iter() {
                    paths.insert(agent, last_node.paths[agent].clone());
                }
                Ok(paths)
            }
            Err(error) => Err(Box::new(error)),
        }
    }
}

#[cfg(test)]
mod tests;
