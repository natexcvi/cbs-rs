use super::*;
use proptest::{
    collection::{hash_map, vec},
    prelude::*,
};
use rstest::rstest;

#[test]
fn test_path_finding() {
    let grid = Grid::new(
        5,
        5,
        Grid::to_conditional_obstacles(
            vec![
                LocationTime {
                    location: (1, 1),
                    time: 0,
                },
                LocationTime {
                    location: (1, 2),
                    time: 0,
                },
                LocationTime {
                    location: (1, 3),
                    time: 0,
                },
                LocationTime {
                    location: (2, 1),
                    time: 0,
                },
                LocationTime {
                    location: (2, 2),
                    time: 0,
                },
                LocationTime {
                    location: (2, 3),
                    time: 0,
                },
                LocationTime {
                    location: (3, 1),
                    time: 0,
                },
                LocationTime {
                    location: (3, 2),
                    time: 0,
                },
                LocationTime {
                    location: (3, 3),
                    time: 0,
                },
            ]
            .into_iter()
            .collect(),
        ),
        (4, 4),
    );
    let start = LocationTime {
        location: (0, 0),
        time: 0,
    };
    let heuristic = heuristic::ManhattanDistance::new(Rc::new(grid.clone()));
    let h = heuristic.h(&start);
    let empty_cat = HashSet::new();
    let start_node = PathFindingNode::new(start, 0.0, h as f64, &grid, &empty_cat, &heuristic);
    let solution = a_star(start_node).expect("No path found");
    assert_eq!(
        solution.path[solution.path.len() - 1].loc_time.location,
        (4, 4)
    );
    assert_eq!(solution.path[solution.path.len() - 1].loc_time.time, 8);
    assert_eq!(solution.path.len(), 9);
}

fn is_valid_move(from: &LocationTime, to: &LocationTime) -> bool {
    let (from_x, from_y) = from.location;
    let (to_x, to_y) = to.location;
    let dx = (from_x as i32 - to_x as i32).abs();
    let dy = (from_y as i32 - to_y as i32).abs();
    let dt = (from.time as i32 - to.time as i32).abs();
    (dx == 1 && dy == 0 && dt == 1)
        || (dx == 0 && dy == 1 && dt == 1)
        || (dx == 0 && dy == 0 && dt == 1)
}

prop_compose! {
    fn empty_grid(max_size: i32)(width in 1..=max_size, height in 1..=max_size)(
        goal_x in 0..width,
        goal_y in 0..height,
        start_x in 0..width,
        start_y in 0..height,
        width in Just(width),
        height in Just(height),
    ) -> (Grid, LocationTime) {
        (Grid::new(width, height, HashMap::new(), (goal_x, goal_y)), LocationTime::new((start_x, start_y), 0))
    }
}

prop_compose! {
    fn grid_with_obstacles(max_size: i32, max_obstacles: usize)(
        (grid, start) in empty_grid(max_size),
    )(
        obstacles in vec((0..grid.width, 0..grid.height), 0..max_obstacles as usize),
        grid in Just(grid),
        start in Just(start),
    ) -> (Grid, LocationTime) {
        let obstacles = obstacles.into_iter().map(|(x, y)| LocationTime::new((x, y), -1)).collect();
        let grid = Grid::new(grid.width, grid.height, Grid::to_conditional_obstacles(obstacles), grid.goal);
        (grid, start)
    }
}

#[derive(Debug, Clone)]
enum HeuristicType {
    ManhattanDistance,
    TrueDistance,
}

fn heuristic_strategy() -> impl Strategy<Value = HeuristicType> {
    prop_oneof![
        Just(HeuristicType::ManhattanDistance),
        Just(HeuristicType::TrueDistance),
    ]
}

proptest! {
    #[rstest]
    fn test_path_validity((grid, start) in empty_grid(100), heuristic_type in heuristic_strategy()) {
        let heuristic = match heuristic_type {
            HeuristicType::ManhattanDistance => Box::new(heuristic::ManhattanDistance::new(Rc::new(grid.clone()))) as Box<dyn Heuristic<LocationTime>>,
            HeuristicType::TrueDistance => Box::new(heuristic::TrueDistance::new(Rc::new(grid.clone()), start.location)) as Box<dyn Heuristic<LocationTime>>,
        };
        let h = heuristic.h(&start);
        let empty_cat = HashSet::new();
        let start_node = PathFindingNode::new(start, 0.0, h as f64, &grid, &empty_cat, heuristic.as_ref());
        let solution = a_star(start_node).expect("No path found");
        let mut prev = solution.path[0];
        for node in solution.path.iter().skip(1) {
            assert!(is_valid_move(&prev.loc_time, &node.loc_time), "Invalid move");
            prev = *node;
        }
        assert_eq!(solution.path[solution.path.len() - 1].loc_time.location, grid.goal);
    }
}
