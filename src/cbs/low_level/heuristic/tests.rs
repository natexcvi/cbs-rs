use rstest::rstest;

use super::*;

#[rstest]
#[case::simple(
    Grid::new(
        5,
        5,
        Grid::to_conditional_obstacles(
            vec![
                LocationTime {
                    location: (1, 0),
                    time: -1,
                },
                LocationTime {
                    location: (1, 1),
                    time: -1,
                },
                LocationTime {
                    location: (1, 2),
                    time: -1,
                },
                LocationTime {
                    location: (1, 3),
                    time: -1,
                },
            ]
            .into_iter()
            .collect(),
        ),
        (2, 0),
    ),
    (0, 0),
    10.0,
)]
#[case::non_start_node(
    Grid::new(
        5,
        5,
        Grid::to_conditional_obstacles(
            vec![
                LocationTime {
                    location: (1, 0),
                    time: -1,
                },
                LocationTime {
                    location: (1, 1),
                    time: -1,
                },
                LocationTime {
                    location: (1, 2),
                    time: -1,
                },
                LocationTime {
                    location: (1, 3),
                    time: -1,
                },
                LocationTime {
                    location: (3, 0),
                    time: -1,
                },
            ]
            .into_iter()
            .collect(),
        ),
        (2, 0),
    ),
    (4, 0),
    4.0,
)]
fn test_true_distance(
    #[case] grid: Grid,
    #[case] query: (i32, i32),
    #[case] true_distance: f64,
) {
    let td = TrueDistance::new(Rc::new(grid), (0, 0));
    let h = td.h(&LocationTime {
        location: query,
        time: 0,
    });
    assert_eq!(h, true_distance);
}
