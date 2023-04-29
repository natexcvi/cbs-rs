use super::{
    low_level::{find_shortest_path, Grid, LocationTime},
    optimisations::conflict_prioritisation::pick_conflict,
    search::{a_star, AStarNode},
};
use std::{collections::HashMap, error::Error, fmt, hash::Hash};

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
    paths: HashMap<&'a Agent, Path>,
    conflicts: Vec<Box<Conflict<'a>>>,
    scenario: &'a Grid,
    conflict_picker:
        fn(&Grid, &HashMap<&Agent, Path>, &Vec<Box<Conflict<'a>>>) -> Option<Box<Conflict<'a>>>,
    post_expanded_callback: fn(Vec<Box<Self>>) -> Option<Vec<Box<Self>>>,
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
            conflict_picker: |_, _, conflicts| Some(conflicts[0].clone()),
            post_expanded_callback: |expanded| Some(expanded), // TODO: replace with optimization
        };
        ctn.conflict_picker = pick_conflict;
        ctn.compute_paths();
        ctn.compute_conflicts();
        ctn
    }

    fn compute_conflicts(&mut self) {
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
        match *conflict {
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
                    )));
                }
            }
        }
        (self.post_expanded_callback)(expanded)
    }
}

#[derive(Debug)]
pub enum CBSError {
    AlreadySolved,
}

impl fmt::Display for CBSError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CBSError::AlreadySolved => write!(f, "CBS instance already solved"),
        }
    }
}

impl Error for CBSError {}

pub struct CBS<'a> {
    scenario: &'a Grid,
    agents: Vec<&'a Agent>,
    solved: bool,
    pub high_level_expanded: usize,
    pub low_level_expanded: usize,
}

impl<'a> CBS<'a> {
    pub fn new(scenario: &'a Grid, agents: Vec<&'a Agent>) -> Self {
        CBS {
            scenario,
            agents,
            high_level_expanded: 0,
            low_level_expanded: 0,
            solved: false,
        }
    }

    pub fn solve(&mut self) -> Result<HashMap<&Agent, Path>, Box<dyn Error>> {
        if self.solved {
            return Err(Box::new(CBSError::AlreadySolved));
        }
        let root = ConflictTreeNode::new(
            self.agents.clone(),
            Vec::<Box<Constraint>>::new(),
            HashMap::<&Agent, Vec<(i32, i32)>>::new(),
            self.scenario,
        );
        let solution = a_star(root);
        self.solved = true;
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
