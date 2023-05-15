use std::{
    collections::{HashMap, HashSet},
    hash::Hash,
};

use super::search::{a_star, AStarNode};

#[derive(Debug, Eq, Clone, Copy)]
pub struct LocationTime {
    pub location: (i32, i32),
    pub time: i32,
}

impl Hash for LocationTime {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.location.hash(state);
        self.time.hash(state);
    }
}

impl PartialEq for LocationTime {
    fn eq(&self, other: &Self) -> bool {
        self.location == other.location && self.time == other.time
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Hash)]
struct Obstacle {
    loc_time: LocationTime,
    coming_from: Option<(i32, i32)>,
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Grid {
    pub width: i32,
    pub height: i32,
    pub obstacles: HashMap<LocationTime, Vec<(i32, i32)>>,
    pub goal: (i32, i32),
}

impl Hash for Grid {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.width.hash(state);
        self.height.hash(state);
        self.obstacles.iter().for_each(|(k, v)| {
            k.hash(state);
            v.hash(state);
        });
        self.goal.hash(state);
    }
}

impl Grid {
    pub fn new(
        width: i32,
        height: i32,
        obstacles: HashMap<LocationTime, Vec<(i32, i32)>>,
        goal: (i32, i32),
    ) -> Grid {
        Grid {
            width,
            height,
            obstacles,
            goal,
        }
    }

    pub fn to_conditional_obstacles(
        obstacles: Vec<LocationTime>,
    ) -> HashMap<LocationTime, Vec<(i32, i32)>> {
        obstacles
            .into_iter()
            .map(|loc_time| (loc_time, Vec::new()))
            .collect()
    }

    pub fn is_valid_location(&self, location: &(i32, i32), prev_location: &(i32, i32)) -> bool {
        self.is_valid_location_time(
            &LocationTime {
                location: *location,
                time: -1,
            },
            prev_location,
        )
    }

    pub fn is_valid_location_time(
        &self,
        loc_time: &LocationTime,
        prev_location: &(i32, i32),
    ) -> bool {
        loc_time.location.0 >= 0
            && loc_time.location.0 < self.width
            && loc_time.location.1 >= 0
            && loc_time.location.1 < self.height
            // permament obstacles
            && !self.is_obstacle(&LocationTime {
                location: loc_time.location,
                time: -1,
            }, prev_location)
            // dynamic obstacles
            && !self.is_obstacle(loc_time, prev_location)
    }

    fn is_obstacle(&self, loc_time: &LocationTime, prev_location: &(i32, i32)) -> bool {
        let coming_from = self.obstacles.get(loc_time);
        match coming_from {
            Some(coming_from) => coming_from.is_empty() || coming_from.contains(&prev_location),
            None => false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PathFindingNode<'a> {
    loc_time: LocationTime,
    g: f64,
    h: f64,
    grid: &'a Grid,
}

impl PartialEq for PathFindingNode<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.loc_time == other.loc_time //&& self.grid == other.grid
    }
}

impl Eq for PathFindingNode<'_> {}

impl Hash for PathFindingNode<'_> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.loc_time.hash(state);
    }
}

impl<'a> PathFindingNode<'a> {
    fn new(loc_time: LocationTime, g: f64, h: f64, grid: &'a Grid) -> PathFindingNode<'a> {
        PathFindingNode {
            loc_time,
            g,
            h,
            grid,
        }
    }
}

impl AStarNode<'_> for PathFindingNode<'_> {
    fn g(&self) -> f64 {
        self.g
    }

    fn h(&self) -> f64 {
        self.h
    }

    fn is_goal(&self) -> bool {
        self.loc_time.location == self.grid.goal
    }

    fn tie_breaker(&self, other: &Self) -> std::cmp::Ordering {
        self.loc_time
            .time
            .cmp(&other.loc_time.time)
            .then(self.loc_time.location.cmp(&other.loc_time.location))
    }

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        let expanded = vec![(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
            .iter()
            .map(|(x, y)| LocationTime {
                location: (self.loc_time.location.0 + x, self.loc_time.location.1 + y),
                time: self.loc_time.time + 1,
            })
            .filter(|neighbour| {
                self.grid
                    .is_valid_location_time(&neighbour, &self.loc_time.location)
            })
            .map(|neighbour| -> Box<PathFindingNode> {
                let h = (neighbour.location.0 - self.grid.goal.0).abs()
                    + (neighbour.location.1 - self.grid.goal.1).abs();
                Box::new(PathFindingNode::new(
                    neighbour,
                    self.g + 1.0,
                    h as f64,
                    self.grid,
                ))
            })
            .collect::<Vec<Box<Self>>>();
        Some(expanded)
    }

    fn id(&self) -> String {
        format!("{:?}", self.loc_time)
    }
}

pub fn find_shortest_path(grid: Grid, start: LocationTime) -> Option<(Vec<LocationTime>, usize)> {
    let h = (start.location.0 - grid.goal.0).abs() + (start.location.1 - grid.goal.1).abs();
    let start_node = PathFindingNode::new(start, 0.0, h as f64, &grid);
    let solution = a_star(start_node).expect("should find path");
    Some((
        solution.path.iter().map(|node| node.loc_time).collect(),
        solution.nodes_generated as usize,
    ))
}

mod tests;
