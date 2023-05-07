use std::{collections::HashSet, hash::Hash};

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

#[derive(Debug, PartialEq, Eq)]
pub struct Grid {
    pub width: i32,
    pub height: i32,
    pub obstacles: HashSet<LocationTime>,
    pub goal: (i32, i32),
    obstacle_map: Vec<Vec<Vec<i32>>>,
}

impl Hash for Grid {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.width.hash(state);
        self.height.hash(state);
        self.obstacles
            .iter()
            .collect::<Vec<&LocationTime>>()
            .hash(state);
        self.goal.hash(state);
    }
}

impl Grid {
    pub fn new(width: i32, height: i32, obstacles: Vec<LocationTime>, goal: (i32, i32)) -> Grid {
        let mut grid = Grid {
            width,
            height,
            obstacles: obstacles.into_iter().collect(),
            goal,
            obstacle_map: vec![vec![Vec::new(); height as usize]; width as usize],
        };
        grid.compute_obstacle_map();
        grid
    }

    fn compute_obstacle_map(&mut self) {
        for obstacle in &self.obstacles {
            self.obstacle_map[obstacle.location.0 as usize][obstacle.location.1 as usize]
                .push(obstacle.time);
        }
        self.obstacle_map.iter_mut().for_each(|row| {
            row.iter_mut().for_each(|cell| {
                cell.sort();
            })
        });
    }

    pub fn is_valid_location(&self, location: &(i32, i32)) -> bool {
        location.0 >= 0
            && location.0 < self.width
            && location.1 >= 0
            && location.1 < self.height
            && !self.obstacle_map[location.0 as usize][location.1 as usize].contains(&-1)
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

    fn expand(&self) -> Option<Vec<Box<Self>>> {
        let expanded = vec![(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
            .iter()
            .map(|(x, y)| LocationTime {
                location: (self.loc_time.location.0 + x, self.loc_time.location.1 + y),
                time: self.loc_time.time + 1,
            })
            .filter(|neighbour| {
                self.grid.is_valid_location(&neighbour.location)
                    && !self
                        .grid
                        .obstacle_map
                        .get(neighbour.location.0 as usize)
                        .unwrap()
                        .get(neighbour.location.1 as usize)
                        .unwrap()
                        .binary_search(&neighbour.time)
                        .is_ok()
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
