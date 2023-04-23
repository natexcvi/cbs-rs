use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::rc::Rc;

pub trait AStarNode {
    fn g(&self) -> f64;
    fn h(&self) -> f64;
    fn expand(&self) -> Vec<Box<Self>>;
}

#[derive(Debug)]
pub enum SearchError {
    InvalidArguments(String),
    NotFound,
}

struct HeapNode<T: AStarNode + Clone> {
    node: T,
    prev: Option<Rc<HeapNode<T>>>,
}

impl<'a, T: AStarNode + Clone> HeapNode<T> {
    fn new(node: T) -> HeapNode<T> {
        HeapNode { node, prev: None }
    }
}

impl<'a, T: AStarNode + Clone> PartialEq for HeapNode<T> {
    fn eq(&self, other: &Self) -> bool {
        self.node.g() == other.node.g()
    }
}

impl<'a, T: AStarNode + Clone> PartialOrd for HeapNode<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (self.node.g() + self.node.h()).partial_cmp(&(other.node.g() + other.node.h()))
    }
}

impl<'a, T: AStarNode + Clone> Eq for HeapNode<T> {}

impl<'a, T: AStarNode + Clone> Ord for HeapNode<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.node.g() + self.node.h()).total_cmp(&(other.node.g() + other.node.h()))
    }
}

pub fn a_star<T: AStarNode + Clone>(start: T) -> Result<T, SearchError> {
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
            return Ok(current.node.clone());
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
