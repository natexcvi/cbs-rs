use super::*;

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
    let heuristic = heuristic::ManhattanDistance::new(&grid);
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
