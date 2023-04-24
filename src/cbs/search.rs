use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::error::Error;
use std::rc::Rc;

pub trait AStarNode<'a> {
    fn g(&'a self) -> f64;
    fn h(&'a self) -> f64;
    fn expand(&'a self) -> Vec<Box<Self>>;
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
        self.node.g() == other.node.g()
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

pub fn a_star<T>(start: T) -> Result<Vec<T>, SearchError>
where
    for<'a> T: AStarNode<'a> + Clone,
{
    let mut frontier = BinaryHeap::<Reverse<HeapNode<T>>>::new();
    frontier.push(Reverse(HeapNode {
        node: start,
        prev: None,
    }));
    loop {
        if frontier.is_empty() {
            return Err(SearchError::NotFound);
        }
        let Reverse(current) = frontier.pop().expect("failed to pop from heap");
        let current = Rc::new(current);
        if current.node.h() == 0.0 {
            return Ok(reconstruct_path(current));
        }
        for neighbor in current.node.expand() {
            frontier.push(Reverse(HeapNode {
                node: *neighbor,
                prev: Some(Rc::clone(&current)),
            }));
        }
    }
}

#[cfg(test)]
mod tests;
