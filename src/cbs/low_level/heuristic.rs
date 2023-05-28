use std::{
    cell::RefCell,
    cmp::Reverse,
    collections::{BinaryHeap, HashMap},
    rc::Rc,
    vec,
};

use crate::cbs::search::{stateful_a_star, AStarNode, HeapNode, SearchError};

use super::{Grid, LocationTime};

pub(crate) trait Heuristic<T> {
    fn h(&self, node: &T) -> f64;
}

type Location = (i32, i32);

#[derive(Clone, Debug)]
struct TrueDistanceNode {
    location: Location,
    time: i32,
    grid: Rc<Grid>,
    heuristic: Rc<ManhattanDistance>,
}

impl std::hash::Hash for TrueDistanceNode {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.location.hash(state);
    }
}

impl Eq for TrueDistanceNode {}

impl PartialEq for TrueDistanceNode {
    fn eq(&self, other: &Self) -> bool {
        self.location == other.location
    }
}

impl AStarNode<'_> for TrueDistanceNode {
    fn g(&self) -> f64 {
        self.time.into()
    }

    fn h(&self) -> f64 {
        self.heuristic.h(&LocationTime {
            location: self.location,
            time: self.time,
        })
    }

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        vec![(0, 1), (0, -1), (1, 0), (-1, 0)]
            .into_iter()
            .map(|(dx, dy)| (self.location.0 + dx, self.location.1 + dy))
            .filter(|(x, y)| self.grid.is_valid_location(&(*x, *y), &self.location))
            .map(|(x, y)| {
                Box::new(TrueDistanceNode {
                    location: (x, y),
                    time: self.time + 1,
                    grid: Rc::clone(&self.grid),
                    heuristic: Rc::clone(&self.heuristic),
                }) as Box<Self>
            })
            .collect::<Vec<Box<Self>>>()
            .into()
    }

    fn is_goal(&self) -> bool {
        false
    }

    fn id(&self) -> String {
        format!("{:?}", self)
    }

    fn tie_breaker(&self, other: &Self) -> std::cmp::Ordering {
        self.time
            .cmp(&other.time)
            .reverse()
            .then_with(|| self.location.cmp(&other.location))
    }
}

pub(crate) struct TrueDistance {
    grid: Rc<Grid>,
    best_g: RefCell<HashMap<Rc<TrueDistanceNode>, f64>>,
    frontier: RefCell<BinaryHeap<Reverse<HeapNode<TrueDistanceNode>>>>,
    heuristic: Rc<ManhattanDistance>,
    max_g: RefCell<f64>,
}

impl TrueDistance {
    pub(crate) fn new(grid: Rc<Grid>, start: Location) -> TrueDistance {
        let td = TrueDistance {
            grid: Rc::clone(&grid),
            frontier: RefCell::new(BinaryHeap::new()),
            best_g: RefCell::new(HashMap::new()),
            heuristic: Rc::new(ManhattanDistance::new(Rc::clone(&grid))),
            max_g: RefCell::new(0.0),
        };
        let aux_grid = Rc::new(Grid::new(
            grid.width,
            grid.height,
            grid.obstacles.clone(),
            start,
        ));
        td.frontier
            .borrow_mut()
            .push(Reverse(HeapNode::new(Rc::new(TrueDistanceNode {
                location: grid.goal,
                time: 0,
                grid: Rc::clone(&aux_grid),
                heuristic: Rc::clone(&td.heuristic),
            }))));
        td.best_g.borrow_mut().insert(
            Rc::new(TrueDistanceNode {
                location: grid.goal,
                time: 0,
                grid: Rc::clone(&aux_grid),
                heuristic: Rc::clone(&td.heuristic),
            }),
            0.0,
        );
        td
    }

    fn compute_h_values(&self, max_dist_from_goal: f64) {
        let result = stateful_a_star(
            &mut self.frontier.borrow_mut(),
            &mut self.best_g.borrow_mut(),
            max_dist_from_goal,
        );
        match result {
            Ok(_) => {}
            Err(SearchError::NotFound) => {}
            Err(e) => panic!("Unexpected error: {}", e),
        }
    }
}

impl Heuristic<LocationTime> for TrueDistance {
    fn h(&self, loc_time: &LocationTime) -> f64 {
        let mut max_dist_increase_factor = 3.0;
        loop {
            let best_g = self.best_g.borrow();
            let h_value = best_g.get(&Rc::new(TrueDistanceNode {
                location: loc_time.location,
                time: loc_time.time,
                grid: Rc::clone(&self.grid),
                heuristic: Rc::clone(&self.heuristic),
            }));
            match h_value {
                Some(h) => return *h,
                None => {
                    drop(best_g);
                    let cur_max = self.max_g.borrow().clone();
                    self.max_g.replace(cur_max + max_dist_increase_factor);
                    max_dist_increase_factor *= 2.0;
                    self.compute_h_values(self.max_g.borrow().clone());
                }
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct ManhattanDistance {
    grid: Rc<Grid>,
}

impl ManhattanDistance {
    pub(crate) fn new(grid: Rc<Grid>) -> ManhattanDistance {
        ManhattanDistance { grid }
    }
}

impl Heuristic<LocationTime> for ManhattanDistance {
    fn h(&self, loc_time: &LocationTime) -> f64 {
        (loc_time.location.0 - self.grid.goal.0).abs() as f64
            + (loc_time.location.1 - self.grid.goal.1).abs() as f64
    }
}

#[cfg(test)]
mod tests;
