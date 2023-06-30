use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
    fs::OpenOptions,
    rc::Rc,
};

use crate::cbs::{
    low_level::LocationTime,
    mdd::{mdd, merge_mdds},
    vertex_cover::{min_vertex_cover, MVCGraph},
};

use super::{Agent, ConflictTreeNode, Path};

pub trait Heuristic {
    fn h(&self, node: &ConflictTreeNode<'_>) -> f64;
}

#[derive(Debug)]
struct DependencyEdge<'a> {
    from: &'a Agent,
    to: &'a Agent,
    weight: f64,
}

impl std::hash::Hash for DependencyEdge<'_> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.from.hash(state);
        self.to.hash(state);
    }
}

impl PartialEq for DependencyEdge<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.from == other.from && self.to == other.to && self.weight == other.weight
    }
}

impl Eq for DependencyEdge<'_> {}

impl<'a> DependencyEdge<'a> {
    fn new(from: &'a Agent, to: &'a Agent, weight: f64) -> Self {
        Self { from, to, weight }
    }
}

type DependencyGraph<'a> = HashSet<DependencyEdge<'a>>;
type AgentWithConstraints = (String, Vec<(LocationTime, Vec<(i32, i32)>)>);
type AgentWithConstraintsPair = (Rc<AgentWithConstraints>, Rc<AgentWithConstraints>);

pub(crate) struct DGHeuristic {
    dependency_edges: RefCell<HashMap<AgentWithConstraintsPair, f64>>,
}

impl Heuristic for DGHeuristic {
    fn h(&self, node: &ConflictTreeNode<'_>) -> f64 {
        self.compute(node)
    }
}

impl DGHeuristic {
    pub(crate) fn new() -> Self {
        Self {
            dependency_edges: RefCell::new(HashMap::new()),
        }
    }

    fn get_dependency_weight(
        &self,
        agent: Rc<AgentWithConstraints>,
        other_agent: Rc<AgentWithConstraints>,
    ) -> Option<f64> {
        self.dependency_edges
            .borrow()
            .get(&(agent, other_agent))
            .copied()
    }

    pub(crate) fn compute(&self, node: &ConflictTreeNode<'_>) -> f64 {
        let mut graph = DependencyGraph::new();
        let mut mdds: HashMap<&Agent, Vec<Vec<(i32, i32)>>> = HashMap::new();
        let mut agents_with_constraints: HashMap<&&Agent, Rc<AgentWithConstraints>> =
            HashMap::new();
        for (i, agent) in node.agents.iter().enumerate() {
            for j in (i + 1)..node.agents.len() {
                let other_agent = &node.agents[j];
                if !agents_with_constraints.contains_key(agent) {
                    agents_with_constraints
                        .insert(agent, self.to_agent_with_constraints(agent, node));
                }
                if !agents_with_constraints.contains_key(other_agent) {
                    agents_with_constraints.insert(
                        other_agent,
                        self.to_agent_with_constraints(other_agent, node),
                    );
                }
                if let Some(dep_weight) = self.get_dependency_weight(
                    Rc::clone(&agents_with_constraints[agent]),
                    Rc::clone(&agents_with_constraints[other_agent]),
                ) {
                    if dep_weight > 0.0 {
                        graph.insert(DependencyEdge::new(agent, other_agent, dep_weight));
                    }
                    continue;
                }
                let c = (node
                    .paths
                    .get(agent)
                    .unwrap()
                    .len()
                    .max(node.paths.get(other_agent).unwrap().len())
                    as i32)
                    - 1;
                if !mdds.contains_key(agent) {
                    mdds.insert(agent, compute_agent_mdd(node, agent));
                }
                if !mdds.contains_key(other_agent) {
                    mdds.insert(other_agent, compute_agent_mdd(node, other_agent));
                }
                let mdd1 = mdds.get(agent).expect("should have been computed");
                let mdd2 = mdds.get(other_agent).expect("should have been computed");
                let joint_mdd = merge_mdds(&mdd1, &mdd2, c);
                let mut dep_weight = 0.0;
                if is_joint_mdd_empty(&joint_mdd, agent, other_agent) {
                    log::debug!(
                        "agent {:?} and agent {:?} have a dependency",
                        agent,
                        other_agent
                    );
                    graph.insert(DependencyEdge::new(agent, other_agent, 1.0));
                    dep_weight = 1.0;
                }
                self.cache_dependency_weight(
                    Rc::clone(&agents_with_constraints[agent]),
                    Rc::clone(&agents_with_constraints[other_agent]),
                    dep_weight,
                );
            }
        }
        let mvc = find_mvc(&graph);
        let h = mvc.len() as f64;
        h
    }

    fn cache_dependency_weight(
        &self,
        agent: Rc<AgentWithConstraints>,
        other_agent: Rc<AgentWithConstraints>,
        weight: f64,
    ) {
        self.dependency_edges
            .borrow_mut()
            .insert((agent, other_agent), weight);
    }
    fn to_agent_with_constraints(
        &self,
        agent: &&Agent,
        node: &ConflictTreeNode<'_>,
    ) -> Rc<AgentWithConstraints> {
        let mut agent_with_constraints = (
            agent.id.clone(),
            node.constraints_to_obstacles(agent)
                .iter()
                .map(|(loc, coming_from)| (loc.clone(), coming_from.clone()))
                .collect::<Vec<_>>(),
        );
        agent_with_constraints
            .1
            .sort_by_key(|(loc, _)| (loc.time, loc.location));
        Rc::new(agent_with_constraints)
    }
}

fn compute_agent_mdd(node: &ConflictTreeNode<'_>, agent: &&Agent) -> Vec<Vec<(i32, i32)>> {
    let c = (node.paths.get(agent).unwrap().len() as i32) - 1;
    let mut scenario = node.scenario.clone();
    scenario
        .obstacles
        .extend(node.constraints_to_obstacles(agent));
    let agent_mdd = mdd(&agent, &scenario, c).unwrap();
    agent_mdd
}

fn is_joint_mdd_empty(
    joint_mdd: &Vec<Vec<((i32, i32), (i32, i32))>>,
    agent: &&Agent,
    other_agent: &&Agent,
) -> bool {
    !joint_mdd
        .last()
        .unwrap()
        .contains(&(agent.goal, other_agent.goal))
        || joint_mdd.iter().any(|layer| layer.is_empty())
}

fn find_mvc<'a>(graph: &'a HashSet<DependencyEdge<'a>>) -> Vec<Rc<&'a Agent>> {
    let mut mvc_graph = MVCGraph::new();
    graph.iter().for_each(|edge| {
        mvc_graph.add_edge(Rc::new(edge.from), Rc::new(edge.to));
    });
    let mut k = 1;
    loop {
        let mvc = min_vertex_cover(&mvc_graph, k);
        if mvc.is_some() {
            return mvc.unwrap();
        }
        if k > graph.len() {
            panic!(
                "{}",
                format!(
                    "graph has {} vertices, so MVC of size at most {} should exist",
                    graph.len(),
                    k
                )
            );
        }
        k = 2 * k;
    }
}

pub(crate) struct ZeroHeuristic {}

impl ZeroHeuristic {
    pub(crate) fn new() -> Self {
        Self {}
    }
}

impl Heuristic for ZeroHeuristic {
    fn h(&self, _node: &ConflictTreeNode<'_>) -> f64 {
        0.0
    }
}

#[cfg(test)]
mod tests;
