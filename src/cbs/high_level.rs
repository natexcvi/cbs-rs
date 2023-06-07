use super::{
    low_level::{AStarLowLevelSolver, Grid, LocationTime, LowLevelSolver},
    search::AStarNode,
};
use std::{
    collections::{HashMap, HashSet},
    hash::Hash,
    rc::Rc,
};

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
pub struct VertexConflict<'a> {
    pub agent1: &'a Agent,
    pub agent2: &'a Agent,
    pub time: i32,
    pub location: (i32, i32),
}

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
pub struct EdgeConflict<'a> {
    pub agent1: &'a Agent,
    pub agent2: &'a Agent,
    pub time: i32,
    pub location1: (i32, i32),
    pub location2: (i32, i32),
}

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
pub enum Conflict<'a> {
    Vertex(VertexConflict<'a>),
    Edge(EdgeConflict<'a>),
}

#[derive(Clone, Eq, PartialEq, Debug, Hash)]
pub struct Constraint<'a> {
    agent: &'a Agent,
    time: i32,
    location: (i32, i32),
    prev_location: Option<(i32, i32)>,
}

impl<'a> Constraint<'a> {
    pub fn agent(&self) -> &Agent {
        self.agent
    }

    pub fn time(&self) -> i32 {
        self.time
    }

    pub fn location(&self) -> (i32, i32) {
        self.location
    }
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
    pub(crate) constraints: Vec<Box<Constraint<'a>>>,
    pub(crate) agents: Vec<&'a Agent>,
    pub(crate) paths: HashMap<&'a Agent, Path>,
    pub(crate) conflicts: Vec<Box<Conflict<'a>>>,
    pub(crate) scenario: &'a Grid,
    conflict_picker:
        fn(&Grid, &HashMap<&Agent, Path>, &Vec<Box<Conflict<'a>>>) -> Option<Box<Conflict<'a>>>,
    post_expanded_callback: fn(&Self, &Conflict<'a>, Vec<Box<Self>>) -> Option<Vec<Box<Self>>>,
    node_preprocessor: Rc<dyn CTNodePreprocessor>,
    use_conflict_avoidance_table: bool,
    low_level_generated: usize,
    low_level_solver: &'a AStarLowLevelSolver,
}

impl<'a> std::fmt::Debug for ConflictTreeNode<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConflictTreeNode")
            .field("constraints", &self.constraints)
            .field("agents", &self.agents)
            .field("paths", &self.paths)
            .field("conflicts", &self.conflicts)
            .field("scenario", &self.scenario)
            .field("low_level_generated", &self.low_level_generated)
            .finish()
    }
}

impl PartialEq for ConflictTreeNode<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.constraints == other.constraints
            && self.agents == other.agents
            && self.paths == other.paths
            && self.conflicts == other.conflicts
            && self.scenario == other.scenario
    }
}

impl Eq for ConflictTreeNode<'_> {}

impl Hash for ConflictTreeNode<'_> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.constraints.hash(state);
        self.agents.hash(state);
        self.conflicts.hash(state);
        self.scenario.hash(state);
    }
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
        node_preprocessor: Option<Rc<dyn CTNodePreprocessor>>,
        use_conflict_avoidance_table: bool,
        low_level_solver: &'a AStarLowLevelSolver,
    ) -> ConflictTreeNode<'a> {
        let mut ctn = ConflictTreeNode {
            constraints,
            agents,
            paths: precomputed_paths,
            conflicts: Vec::<Box<Conflict>>::new(),
            scenario,
            conflict_picker: |_, _, conflicts| Some(conflicts[0].clone()),
            post_expanded_callback: |_, _, expanded| Some(expanded), // TODO: replace with optimization
            node_preprocessor: Rc::new(IdentityPreprocessor::new()),
            low_level_generated: 0,
            use_conflict_avoidance_table,
            low_level_solver,
        };
        if let Some(pick_conflict) = conflict_picker {
            ctn.conflict_picker = pick_conflict;
        }
        if let Some(callback) = post_expanded_callback {
            ctn.post_expanded_callback = callback;
        }
        if let Some(preprocessor) = node_preprocessor {
            ctn.node_preprocessor = preprocessor;
        }
        Rc::clone(&ctn.node_preprocessor).preprocess(&mut ctn);
        log::debug!(
            "Agents left to plan after preprocessing: {}/{}",
            ctn.agents.len() - ctn.paths.len(),
            ctn.agents.len()
        );
        log::debug!(
            "Agents left to plan for: {:?}",
            ctn.agents
                .iter()
                .filter(|a| !ctn.paths.contains_key(*a))
                .collect::<Vec<_>>()
        );
        let t0 = std::time::Instant::now();
        ctn.compute_paths();
        log::debug!("Time to compute paths: {:?}", t0.elapsed());
        let t0 = std::time::Instant::now();
        ctn.compute_conflicts();
        log::debug!("Time to compute conflicts: {:?}", t0.elapsed());
        log::debug!("Number of conflicts: {}", ctn.conflicts.len());
        log::debug!("Number of constraints: {}", ctn.constraints.len());
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
                let current_location_agents = agent_locations
                    .entry(location)
                    .or_insert(Vec::<&Agent>::new());
                if !current_location_agents.contains(&agent) {
                    current_location_agents.push(agent);
                }
                if time_step > 0 {
                    let prev_location = self.paths[agent][time_step - 1];
                    if prev_location != location {
                        agent_locations
                            .entry(prev_location)
                            .or_default()
                            .retain(|a| a != agent);
                    }
                }
            }
            for (location, agents) in agent_locations.iter() {
                for (j, agent1) in agents.iter().enumerate() {
                    for agent2 in agents[..j].iter() {
                        if agents.len() > 1 {
                            conflicts.push(Box::new(Conflict::Vertex(VertexConflict {
                                agent1,
                                agent2,
                                time: time_step as i32,
                                location: location.clone(),
                            })));
                        }
                    }
                    if time_step == 0 || time_step >= self.paths[agent1].len() {
                        continue;
                    }
                    for agent2 in agent_locations
                        .get(&self.paths[agent1][time_step - 1])
                        .unwrap()
                    {
                        if time_step >= self.paths[agent2].len()
                            || agent1 == agent2
                            || &self.paths[agent1][time_step - 1] > location
                        {
                            continue;
                        }
                        if &self.paths[agent2][time_step - 1] == location {
                            conflicts.push(Box::new(Conflict::Edge(EdgeConflict {
                                agent1,
                                agent2,
                                time: time_step as i32,
                                location1: location.clone(),
                                location2: self.paths[agent1][time_step - 1],
                            })));
                        }
                    }
                }
            }
        }
        self.conflicts = conflicts;
    }

    fn build_conflict_avoidance_table(&self) -> HashSet<LocationTime> {
        let mut conflict_avoidance_table = HashSet::<LocationTime>::new();
        for (_, path) in self.paths.iter() {
            Self::update_conflict_avoidance_table(&mut conflict_avoidance_table, path);
        }
        conflict_avoidance_table
    }

    fn update_conflict_avoidance_table(
        conflict_avoidance_table: &mut HashSet<LocationTime>,
        path: &Path,
    ) {
        for (i, location) in path.iter().enumerate() {
            if i == 0 {
                continue;
            }
            conflict_avoidance_table.insert(LocationTime {
                location: *location,
                time: i as i32,
            });
        }
    }

    fn compute_paths(&mut self) {
        let mut conflict_avoidance_table = if self.use_conflict_avoidance_table {
            self.build_conflict_avoidance_table()
        } else {
            HashSet::new()
        };
        for agent in self.agents.iter() {
            if self.paths.contains_key(agent) {
                continue;
            }
            let mut obstacles = self.constraints_to_obstacles(agent);
            self.scenario.obstacles.iter().for_each(|(loc, prevs)| {
                obstacles
                    .entry(*loc)
                    .or_insert(vec![])
                    .extend(prevs.clone());
            });
            let path = self.low_level_solver.find_shortest_path(
                agent.id.clone(),
                Grid::new(
                    self.scenario.width,
                    self.scenario.height,
                    obstacles.into_iter().collect(),
                    agent.goal,
                ),
                LocationTime {
                    location: agent.start,
                    time: 0,
                },
                &conflict_avoidance_table,
            );
            self.paths.insert(
                agent,
                path.map(|(path, nodes_generated)| {
                    self.low_level_generated += nodes_generated;
                    path
                })
                .unwrap()
                .iter()
                .map(|n| n.location)
                .collect(),
            );
            if self.use_conflict_avoidance_table {
                Self::update_conflict_avoidance_table(
                    &mut conflict_avoidance_table,
                    &self.paths[agent],
                );
            }
        }
    }

    pub(crate) fn constraints_to_obstacles(
        &self,
        agent: &&Agent,
    ) -> HashMap<LocationTime, Vec<(i32, i32)>> {
        self.constraints
            .iter()
            .filter(|c| c.agent == *agent)
            .map(|c| {
                (
                    LocationTime {
                        location: c.location,
                        time: c.time,
                    },
                    match c.prev_location {
                        Some(prev_location) => vec![prev_location],
                        None => vec![],
                    },
                )
            })
            .collect()
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

    fn tie_breaker(&self, other: &Self) -> std::cmp::Ordering {
        self.constraints
            .len()
            .cmp(&other.constraints.len())
            .reverse()
            .then_with(|| self.conflicts.len().cmp(&other.conflicts.len()))
    }

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        let mut expanded = Vec::<Box<Self>>::new();
        if self.conflicts.is_empty() {
            return Some(expanded);
        }
        let conflict = (self.conflict_picker)(self.scenario, &self.paths, &self.conflicts)?;
        log::debug!("Expanding conflict: {:?}", conflict);
        match *conflict.clone() {
            Conflict::Vertex(vc) => {
                for agent in vec![vc.agent1, vc.agent2] {
                    let constraint = Constraint {
                        agent,
                        time: vc.time,
                        location: vc.location,
                        prev_location: None,
                    };
                    if self.constraints.contains(&Box::new(constraint.clone())) {
                        continue;
                    }
                    let mut new_constraints = self.constraints.clone();
                    new_constraints.push(Box::new(constraint));
                    log::debug!("Current constraints: {:?}", self.constraints);
                    log::debug!("New constraints: {:?}", new_constraints);
                    let mut new_paths = self.paths.clone();
                    new_paths.remove(agent);
                    expanded.push(Box::new(ConflictTreeNode::new(
                        self.agents.clone(),
                        new_constraints,
                        new_paths,
                        self.scenario,
                        Some(self.conflict_picker),
                        Some(self.post_expanded_callback),
                        Some(Rc::clone(&self.node_preprocessor)),
                        self.use_conflict_avoidance_table,
                        self.low_level_solver,
                    )));
                }
            }
            Conflict::Edge(ec) => {
                for (i, agent) in vec![ec.agent1, ec.agent2].iter().enumerate() {
                    let constraint = Constraint {
                        agent,
                        time: ec.time,
                        location: if i == 0 { ec.location1 } else { ec.location2 },
                        prev_location: Some(if i == 0 { ec.location2 } else { ec.location1 }),
                    };
                    let mut vertex_equiv_constraint = constraint.clone();
                    vertex_equiv_constraint.prev_location = None;
                    if self
                        .constraints
                        .contains(&Box::new(vertex_equiv_constraint))
                        || self.constraints.contains(&Box::new(constraint.clone()))
                    {
                        continue;
                    }
                    let mut new_constraints = self.constraints.clone();
                    new_constraints.push(Box::new(constraint));
                    log::debug!("Current constraints: {:?}", self.constraints);
                    log::debug!("New constraints: {:?}", new_constraints);
                    let mut new_paths = self.paths.clone();
                    new_paths.remove(agent);
                    expanded.push(Box::new(ConflictTreeNode::new(
                        self.agents.clone(),
                        new_constraints,
                        new_paths,
                        self.scenario,
                        Some(self.conflict_picker),
                        Some(self.post_expanded_callback),
                        Some(Rc::clone(&self.node_preprocessor)),
                        self.use_conflict_avoidance_table,
                        self.low_level_solver,
                    )));
                }
            }
        }
        (self.post_expanded_callback)(self, &conflict, expanded)
    }

    fn id(&self) -> String {
        format!(
            "ConflictTreeNode({:?}, {:?}, {:?})",
            self.paths, self.conflicts, self.constraints
        )
    }
}

pub trait CTNodePreprocessor {
    fn preprocess(&self, node: &mut ConflictTreeNode);
}

struct IdentityPreprocessor {}

impl IdentityPreprocessor {
    fn new() -> Self {
        Self {}
    }
}

impl CTNodePreprocessor for IdentityPreprocessor {
    fn preprocess(&self, _node: &mut ConflictTreeNode) {}
}

#[cfg(test)]
mod tests;
