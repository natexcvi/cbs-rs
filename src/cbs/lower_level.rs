use super::search::{a_star, AStarNode};

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct LocationTime {
    pub location: (i32, i32),
    pub time: i32,
}

#[derive(Debug)]
pub struct Grid {
    pub width: i32,
    pub height: i32,
    pub obstacles: Vec<LocationTime>,
    pub goal: (i32, i32),
}

#[derive(Debug, Clone, Copy)]
struct PathFindingNode<'a> {
    loc_time: LocationTime,
    g: f64,
    h: f64,
    grid: &'a Grid,
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

impl AStarNode for PathFindingNode<'_> {
    fn g(&self) -> f64 {
        self.g
    }

    fn h(&self) -> f64 {
        self.h
    }

    fn expand(&self) -> Vec<Box<Self>> {
        let mut expanded = Vec::<Box<Self>>::new();
        let mut neigbours: Vec<LocationTime> = vec![
            LocationTime {
                location: (self.loc_time.location.0 - 1, self.loc_time.location.1),
                time: self.loc_time.time + 1,
            },
            LocationTime {
                location: (self.loc_time.location.0 + 1, self.loc_time.location.1),
                time: self.loc_time.time + 1,
            },
            LocationTime {
                location: (self.loc_time.location.0, self.loc_time.location.1 - 1),
                time: self.loc_time.time + 1,
            },
            LocationTime {
                location: (self.loc_time.location.0, self.loc_time.location.1 + 1),
                time: self.loc_time.time + 1,
            },
        ];
        neigbours.retain(|neighbour| {
            neighbour.location.0 >= 0
                && neighbour.location.0 < self.grid.width
                && neighbour.location.1 >= 0
                && neighbour.location.1 < self.grid.height
                && !self.grid.obstacles.contains(neighbour)
        });
        for neighbour in neigbours {
            let h = (neighbour.location.0 - self.grid.goal.0).abs()
                + (neighbour.location.1 - self.grid.goal.1).abs();
            expanded.push(Box::new(PathFindingNode::new(
                neighbour,
                self.g + 1.0,
                h as f64,
                self.grid,
            )));
        }
        expanded
    }
}

pub fn find_shortest_path(grid: Grid, start: LocationTime) -> Option<Vec<LocationTime>> {
    let h = (start.location.0 - grid.goal.0).abs() + (start.location.1 - grid.goal.1).abs();
    let start_node = PathFindingNode::new(start, 0.0, h as f64, &grid);
    let path = a_star(start_node).expect("No path found");
    Some(path.iter().map(|node| node.loc_time).collect())
}

mod tests;
