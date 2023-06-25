use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
    rc::Rc,
};

use crate::cbs::{
    mdd::{mdd, merge_mdds},
    vertex_cover::{min_vertex_cover, MVCGraph},
};

use super::{Agent, ConflictTreeNode};

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

pub(crate) struct DGHeuristic {}

impl Heuristic for DGHeuristic {
    fn h(&self, node: &ConflictTreeNode<'_>) -> f64 {
        self.compute(node)
    }
}

impl DGHeuristic {
    pub(crate) fn new() -> Self {
        Self {}
    }

    pub(crate) fn compute(&self, node: &ConflictTreeNode<'_>) -> f64 {
        let mut graph = DependencyGraph::new();
        let mut mdds: HashMap<&Agent, Vec<Vec<(i32, i32)>>> = HashMap::new();
        for agent in node.agents.iter() {
            for other_agent in node.agents.iter() {
                if agent == other_agent {
                    continue;
                }
                let c = node
                    .paths
                    .get(agent)
                    .unwrap()
                    .len()
                    .max(node.paths.get(other_agent).unwrap().len()) as i32;
                if !mdds.contains_key(agent) {
                    mdds.insert(agent, mdd(agent, node.scenario, c).unwrap());
                }
                if !mdds.contains_key(other_agent) {
                    mdds.insert(other_agent, mdd(&other_agent, node.scenario, c).unwrap());
                }
                let mdd1 = mdds.get(agent).expect("should have been computed");
                let mdd2 = mdds.get(other_agent).expect("should have been computed");
                let joint_mdd = merge_mdds(&mdd1, &mdd2, c);
                if !joint_mdd
                    .last()
                    .unwrap()
                    .contains(&(agent.goal, other_agent.goal))
                {
                    graph.insert(DependencyEdge::new(agent, other_agent, 1.0));
                }
            }
        }
        let mvc = find_mvc(&graph);
        let h = mvc.len() as f64;
        h
    }
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
