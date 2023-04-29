use super::{
    low_level::{find_shortest_path, Grid, LocationTime},
    search::AStarNode,
};
use std::{collections::HashMap, hash::Hash};

#[derive(Clone, Eq, PartialEq, Debug)]
pub struct VertexConflict<'a> {
    pub agent1: &'a Agent,
    pub agent2: &'a Agent,
    pub time: i32,
    pub location: (i32, i32),
}

#[derive(Clone, Eq, PartialEq, Debug)]
pub struct EdgeConflict<'a> {
    pub agent1: &'a Agent,
    pub agent2: &'a Agent,
    pub time: i32,
    pub location1: (i32, i32),
    pub location2: (i32, i32),
}

#[derive(Clone, Eq, PartialEq, Debug)]
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

#[derive(PartialEq, Eq, Hash, Clone, Debug)]
pub struct Agent {
    pub id: String,
    pub start: (i32, i32),
    pub goal: (i32, i32),
}

#[derive(Clone)]
pub struct ConflictTreeNode<'a> {
    constraints: Vec<Box<Constraint<'a>>>,
    agents: Vec<&'a Agent>,
    pub paths: HashMap<&'a Agent, Path>,
    pub conflicts: Vec<Box<Conflict<'a>>>,
    scenario: &'a Grid,
    conflict_picker:
        fn(&Grid, &HashMap<&Agent, Path>, &Vec<Box<Conflict<'a>>>) -> Option<Box<Conflict<'a>>>,
    post_expanded_callback: fn(&Self, &Conflict<'a>, Vec<Box<Self>>) -> Option<Vec<Box<Self>>>,
}

impl<'a> ConflictTreeNode<'a> {
    pub fn new(
        agents: Vec<&'a Agent>,
        constraints: Vec<Box<Constraint<'a>>>,
        precomputed_paths: HashMap<&'a Agent, Vec<(i32, i32)>>,
        scenario: &'a Grid,
        conflict_picker: Option<
            fn(&Grid, &HashMap<&Agent, Path>, &Vec<Box<Conflict<'a>>>) -> Option<Box<Conflict<'a>>>,
        >,
        post_expanded_callback: Option<
            fn(&Self, &Conflict<'a>, Vec<Box<Self>>) -> Option<Vec<Box<Self>>>,
        >,
    ) -> ConflictTreeNode<'a> {
        let mut ctn = ConflictTreeNode {
            constraints,
            agents,
            paths: precomputed_paths,
            conflicts: Vec::<Box<Conflict>>::new(),
            scenario,
            conflict_picker: |_, _, conflicts| Some(conflicts[0].clone()),
            post_expanded_callback: |_, _, expanded| Some(expanded), // TODO: replace with optimization
        };
        if let Some(pick_conflict) = conflict_picker {
            ctn.conflict_picker = pick_conflict;
        }
        if let Some(callback) = post_expanded_callback {
            ctn.post_expanded_callback = callback;
        }
        ctn.compute_paths();
        ctn.compute_conflicts();
        ctn
    }

    pub fn compute_conflicts(&mut self) {
        let mut conflicts = Vec::<Box<Conflict>>::new();
        let mut agent_locations = HashMap::<(i32, i32), Vec<&Agent>>::new();
        for time_step in 0..self.paths.values().map(|p| p.len()).max().unwrap() {
            for agent in self.agents.iter() {
                if time_step >= self.paths[agent].len() {
                    continue;
                }
                let location = self.paths[agent][time_step];
                agent_locations
                    .entry(location)
                    .or_insert(Vec::<&Agent>::new())
                    .push(agent);
                if time_step > 0 {
                    agent_locations
                        .entry(self.paths[agent][time_step - 1])
                        .or_default()
                        .retain(|a| a != agent);
                }
            }
            for (location, agents) in agent_locations.iter() {
                if agents.len() > 1 {
                    for (j, agent1) in agents.iter().enumerate() {
                        for agent2 in agents[..j].iter() {
                            if time_step > 0
                                && time_step < self.paths[agent2].len()
                                && time_step < self.paths[agent1].len()
                                && self.paths[agent2][time_step]
                                    == self.paths[agent1][time_step - 1]
                                && self.paths[agent2][time_step - 1]
                                    == self.paths[agent1][time_step]
                            {
                                conflicts.push(Box::new(Conflict::Edge(EdgeConflict {
                                    agent1,
                                    agent2,
                                    time: time_step as i32,
                                    location1: self.paths[agent1][time_step - 1],
                                    location2: location.clone(),
                                })));
                            } else {
                                conflicts.push(Box::new(Conflict::Vertex(VertexConflict {
                                    agent1,
                                    agent2,
                                    time: time_step as i32,
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
        let conflict = (self.conflict_picker)(self.scenario, &self.paths, &self.conflicts)?;
        match *conflict.clone() {
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
                        Some(self.conflict_picker),
                        Some(self.post_expanded_callback),
                    )));
                }
            }
            Conflict::Edge(ec) => {
                for agent in vec![ec.agent1, ec.agent2] {
                    // TODO: remove the second constraint which is unneeded
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
                        Some(self.conflict_picker),
                        Some(self.post_expanded_callback),
                    )));
                }
            }
        }
        (self.post_expanded_callback)(self, &conflict, expanded)
    }
}

#[cfg(test)]
mod tests;
