use super::{Grid, LocationTime};

pub(crate) trait Heuristic<T> {
    fn h(&self, node: &T) -> f64;
}

struct TrueDistance<'a> {
    grid: &'a Grid,
}

impl<'a> TrueDistance<'a> {
    fn new(grid: &'a Grid) -> TrueDistance<'a> {
        TrueDistance { grid }
    }
}

impl<'a> Heuristic<LocationTime> for TrueDistance<'a> {
    fn h(&self, loc_time: &LocationTime) -> f64 {
        todo!()
    }
}

pub(crate) struct ManhattanDistance<'a> {
    grid: &'a Grid,
}

impl<'a> ManhattanDistance<'a> {
    pub(crate) fn new(grid: &'a Grid) -> ManhattanDistance<'a> {
        ManhattanDistance { grid }
    }
}

impl<'a> Heuristic<LocationTime> for ManhattanDistance<'a> {
    fn h(&self, loc_time: &LocationTime) -> f64 {
        (loc_time.location.0 - self.grid.goal.0).abs() as f64
            + (loc_time.location.1 - self.grid.goal.1).abs() as f64
    }
}
