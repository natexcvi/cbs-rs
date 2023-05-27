use log::debug;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::error::Error;
use std::hash::Hash;
use std::rc::Rc;

pub trait AStarNode<'a> {
    fn g(&'a self) -> f64;
    fn h(&'a self) -> f64;
    fn expand(&'a self) -> Option<Vec<Box<Self>>>;
    fn is_goal(&'a self) -> bool;
    fn id(&'a self) -> String;
    fn tie_breaker(&'a self, other: &'a Self) -> std::cmp::Ordering;
}

#[derive(Debug)]
pub enum SearchError {
    InvalidArguments(String),
    NotFound,
}

impl std::fmt::Display for SearchError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SearchError::InvalidArguments(s) => write!(f, "Invalid arguments: {}", s),
            SearchError::NotFound => write!(f, "Not found"),
        }
    }
}

impl Error for SearchError {}

struct HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    node: Rc<T>,
    prev: Option<Rc<HeapNode<T>>>,
}

impl<T> HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn new(node: Rc<T>) -> HeapNode<T> {
        HeapNode { node, prev: None }
    }
}

impl<T> PartialEq for HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn eq(&self, other: &Self) -> bool {
        self.node.g() == other.node.g() && self.node.h() == other.node.h()
    }
}

impl<T> PartialOrd for HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (self.node.g() + self.node.h())
            .partial_cmp(&(other.node.g() + other.node.h()))
            .map(|o| {
                if o == std::cmp::Ordering::Equal {
                    self.node
                        .g()
                        .partial_cmp(&other.node.g())
                        .expect("if g() + h() is comparable, g() should be comparable")
                        .reverse()
                        .then_with(|| self.node.tie_breaker(&other.node))
                } else {
                    o
                }
            })
    }
}

impl<T> Eq for HeapNode<T> where for<'a> T: AStarNode<'a> + Clone {}

impl<T> Ord for HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let o = (self.node.g() + self.node.h()).total_cmp(&(other.node.g() + other.node.h()));
        match o {
            std::cmp::Ordering::Equal => self
                .node
                .g()
                .total_cmp(&other.node.g())
                .reverse()
                .then_with(|| self.node.tie_breaker(&other.node)),
            _ => o,
        }
    }
}

pub struct AStarSolution<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    pub path: Vec<T>,
    pub nodes_generated: i32,
}

fn reconstruct_path<T>(mut current: Rc<HeapNode<T>>) -> Vec<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    let mut path = Vec::<T>::new();
    loop {
        path.push((*current.node).clone());
        if let Some(prev) = &current.prev {
            current = Rc::clone(prev);
        } else {
            break;
        }
    }
    path.reverse();
    path
}

pub fn a_star<T>(start: T) -> Result<AStarSolution<T>, SearchError>
where
    for<'a> T: AStarNode<'a> + Clone + std::hash::Hash + Eq,
{
    let t0 = std::time::Instant::now();
    let mut frontier = BinaryHeap::<Reverse<HeapNode<T>>>::new();
    let mut nodes_generated = 0;
    let mut best_g = HashMap::<Rc<T>, f64>::new();
    let start = Rc::new(start);
    let start_g = start.g();
    frontier.push(Reverse(HeapNode::new(Rc::clone(&start))));
    best_g.insert(start, start_g);
    loop {
        if frontier.is_empty() {
            return Err(SearchError::NotFound);
        }
        let Reverse(current) = frontier.pop().expect("heap should not be empty");
        let current = Rc::new(current);
        if current.node.is_goal() {
            debug!(
                "A* took {:?} - {:} nodes generated",
                t0.elapsed(),
                nodes_generated
            );
            return Ok(AStarSolution {
                path: reconstruct_path(Rc::clone(&current)),
                nodes_generated,
            });
        }
        if current.node.g() > *best_g.get(&current.node).unwrap_or(&f64::INFINITY) {
            continue;
        }
        match current.node.expand() {
            Some(expand) => {
                for neighbor in expand {
                    let neighbor = Rc::new(*neighbor);
                    if neighbor.g() >= *best_g.get(&neighbor).unwrap_or(&f64::INFINITY) {
                        continue;
                    }
                    best_g.insert(Rc::clone(&neighbor), neighbor.g());
                    frontier.push(Reverse(HeapNode {
                        node: Rc::clone(&neighbor),
                        prev: Some(Rc::clone(&current)),
                    }));
                    nodes_generated += 1;
                }
            }
            None => {
                frontier.push(Reverse(HeapNode {
                    node: Rc::clone(&current.node),
                    prev: match &current.prev {
                        Some(prev) => Some(Rc::clone(prev)),
                        None => None,
                    },
                }));
            }
        }
    }
}

pub fn dfs<T, S>(
    visited: &mut HashSet<T>,
    result: &mut S,
    processor: &dyn Fn(&mut S, &T, &Option<T>) -> bool,
    on_backtrack: &dyn Fn(&mut S, &T, &Option<T>),
    cur: T,
    parent: Option<T>,
    neighbours: &dyn Fn(&T) -> Vec<T>,
    is_goal: &dyn Fn(&T) -> bool,
) -> bool
where
    T: Hash + Eq + Clone,
{
    let cur_neighbours = neighbours(&cur);
    let should_expand = processor(result, &cur, &parent);
    visited.insert(cur.clone());
    if !should_expand {
        return false;
    }
    if is_goal(&cur) {
        return true;
    }

    for next in cur_neighbours {
        if !visited.contains(&next) {
            if dfs(
                visited,
                result,
                processor,
                on_backtrack,
                next,
                Some(cur.clone()),
                neighbours,
                is_goal,
            ) {
                return true;
            }
            on_backtrack(result, &cur, &parent);
        }
    }
    return false;
}

#[cfg(test)]
mod tests;
