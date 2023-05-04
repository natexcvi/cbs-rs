use super::*;

#[test]
fn test_path_finding() {
    let grid = Grid::new(
        5,
        5,
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
        (4, 4),
    );
    let start = LocationTime {
        location: (0, 0),
        time: 0,
    };
    let h = (start.location.0 - grid.goal.0).abs() + (start.location.1 - grid.goal.1).abs();
    let start_node = PathFindingNode::new(start, 0.0, h as f64, &grid);
    let solution = a_star(start_node).expect("No path found");
    assert_eq!(
        solution.path[solution.path.len() - 1].loc_time.location,
        (4, 4)
    );
    assert_eq!(solution.path[solution.path.len() - 1].loc_time.time, 8);
    assert_eq!(solution.path.len(), 9);
}
