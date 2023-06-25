use std::{
    collections::{HashMap, HashSet},
    rc::Rc,
};

#[derive(Clone)]
pub(crate) struct MVCGraph<T>
where
    T: Eq + std::hash::Hash + Clone,
{
    edges: HashSet<(Rc<T>, Rc<T>)>,
}

impl<'a, T> MVCGraph<T>
where
    T: Eq + std::hash::Hash + Clone,
{
    pub(crate) fn new() -> Self {
        Self {
            edges: HashSet::<(Rc<T>, Rc<T>)>::new(),
        }
    }

    fn empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub(crate) fn add_edge(&mut self, u: Rc<T>, v: Rc<T>) {
        self.edges.insert((u, v));
    }

    pub(crate) fn remove_edge(&mut self, u: Rc<T>, v: Rc<T>) {
        self.edges.remove(&(u, v));
    }

    fn get_neighbours(&self, u: &'a T) -> Option<Vec<Rc<T>>> {
        let mut neighbours = Vec::<Rc<T>>::new();
        for edge in self.edges.iter() {
            if edge.0 == Rc::new(u.clone()) {
                neighbours.push(Rc::clone(&edge.1));
            }
            if edge.1 == Rc::new(u.clone()) {
                neighbours.push(Rc::clone(&edge.0));
            }
        }
        neighbours.dedup();
        match neighbours.is_empty() {
            true => None,
            false => Some(neighbours),
        }
    }

    fn get_some_edge(&self) -> Option<(Rc<T>, Rc<T>)> {
        match self.edges.iter().next() {
            Some(edge) => Some(edge.clone()),
            None => None,
        }
    }

    pub(crate) fn remove_vertex(&mut self, u: Rc<T>) {
        self.edges.retain(|x| x.0 != u && x.1 != u);
    }
}

/// Find a minimum vertex cover of a graph using an iterative
/// compression based algorithm.
pub(crate) fn min_vertex_cover<T>(graph: &MVCGraph<T>, k: usize) -> Option<Vec<Rc<T>>>
where
    T: Eq + std::hash::Hash + std::fmt::Debug + Clone,
{
    if graph.empty() {
        return Some(Vec::<Rc<T>>::new());
    }
    if k == 0 {
        return None;
    }
    let (u, v) = graph
        .get_some_edge()
        .expect("should have checked graph is not empty");
    let mut graph_min_u = graph.clone();
    graph_min_u.remove_vertex(Rc::clone(&u));
    let u_dropped = min_vertex_cover(&graph_min_u, k - 1);
    let cover_with_u = if let Some(mut u_dropped) = u_dropped {
        u_dropped.push(Rc::clone(&u));
        Some(u_dropped)
    } else {
        None
    };
    let mut graph_min_v = graph.clone();
    graph_min_v.remove_vertex(Rc::clone(&v));
    let v_dropped = min_vertex_cover(&graph_min_v, k - 1);
    let cover_with_v = if let Some(mut v_dropped) = v_dropped {
        v_dropped.push(Rc::clone(&v));
        Some(v_dropped)
    } else {
        None
    };
    match (cover_with_u, cover_with_v) {
        (Some(cover_with_u), Some(cover_with_v)) => {
            if cover_with_u.len() < cover_with_v.len() {
                Some(cover_with_u)
            } else {
                Some(cover_with_v)
            }
        }
        (Some(cover_with_u), None) => Some(cover_with_u),
        (None, Some(cover_with_v)) => Some(cover_with_v),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests;
