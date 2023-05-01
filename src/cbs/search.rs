use std::borrow::Borrow;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::error::Error;
use std::rc::Rc;

pub trait AStarNode<'a> {
    fn g(&'a self) -> f64;
    fn h(&'a self) -> f64;
    fn expand(&'a self) -> Option<Vec<Box<Self>>>;
    fn is_goal(&'a self) -> bool;
    fn id(&'a self) -> String;
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
    node: T,
    prev: Option<Rc<HeapNode<T>>>,
}

impl<T> HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn new(node: T) -> HeapNode<T> {
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
        (self.node.g() + self.node.h()).partial_cmp(&(other.node.g() + other.node.h()))
    }
}

impl<T> Eq for HeapNode<T> where for<'a> T: AStarNode<'a> + Clone {}

impl<T> Ord for HeapNode<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.node.g() + self.node.h()).total_cmp(&(other.node.g() + other.node.h()))
    }
}

pub struct AStarSolution<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    pub path: Vec<T>,
    pub nodes_expanded: i32,
}

fn reconstruct_path<T>(mut current: Rc<HeapNode<T>>) -> Vec<T>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    let mut path = Vec::<T>::new();
    loop {
        path.push(current.node.clone());
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
    let mut frontier = BinaryHeap::<Reverse<HeapNode<T>>>::new();
    let mut nodes_expanded = 0;
    let mut best_g = HashMap::<String, f64>::new();
    frontier.push(Reverse(HeapNode {
        node: start,
        prev: None,
    }));
    loop {
        if frontier.is_empty() {
            return Err(SearchError::NotFound);
        }
        let Reverse(current) = frontier.pop().expect("heap should not be empty");
        let current = Rc::new(current);
        if current.node.is_goal() {
            return Ok(AStarSolution {
                path: reconstruct_path(current),
                nodes_expanded,
            });
        }
        match current.node.expand() {
            Some(expand) => {
                for neighbor in expand {
                    if neighbor.g() >= *best_g.get(&neighbor.id()).unwrap_or(&f64::INFINITY) {
                        continue;
                    }
                    best_g.insert(neighbor.id(), neighbor.g());
                    frontier.push(Reverse(HeapNode {
                        node: *neighbor.clone(),
                        prev: Some(Rc::clone(&current)),
                    }));
                    nodes_expanded += 1;
                }
            }
            None => {
                frontier.push(Reverse(HeapNode {
                    node: current.node.clone(),
                    prev: match &current.prev {
                        Some(prev) => Some(Rc::clone(prev)),
                        None => None,
                    },
                }));
            }
        }
    }
}

#[cfg(test)]
mod tests;
