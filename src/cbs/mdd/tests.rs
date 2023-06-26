use crate::cbs::low_level::LocationTime;

use super::*;
use rstest::rstest;
#[rstest]
#[case::simple(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0)),
(0, 0),
(5, 5),
10,
Ok(vec![
    vec![(0,0)],
    vec![(1,0),(0,1)],
    vec![(2,0),(1,1),(0,2)],
    vec![(3,0),(2,1),(1,2),(0,3)],
    vec![(4,0),(3,1),(2,2),(1,3),(0,4)],
    vec![(5,0),(4,1),(3,2),(2,3),(1,4),(0,5)],
    vec![(5,1), (4,2), (3,3), (2,4), (1,5)],
    vec![(5,2), (4,3), (3,4), (2,5)],
    vec![(5,3), (4,4), (3,5)],
    vec![(5,4), (4,5)],
    vec![(5,5)],
])
)]
#[case::non_optimal_path(
crate::cbs::low_level::Grid::new(16, 16, Grid::to_conditional_obstacles(vec![
    LocationTime::new((6, 10), 4),
    LocationTime::new((6, 10), 2),
]), (0, 0)),
(6, 6),
(6, 12),
7,
Ok(vec![
    vec![(6,6)],
    vec![(6,7), (6,6)],
    vec![(6,8), (6,7)],
    vec![(6,9), (6,8)],
    vec![(6,9)],
    vec![(6,10)],
    vec![(6,11)],
    vec![(6,12)],
])
)]
#[case::with_obstacles(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![
    LocationTime::new((1, 1), -1),
    LocationTime::new((1, 2), -1),
    LocationTime::new((1, 3), -1),
]), (0, 0)),
(0, 0),
(5, 5),
10,
Ok(vec![
    vec![(0,0)],
    vec![(1,0),(0,1)],
    vec![(2,0),(0,2)],
    vec![(3,0),(2,1),(0,3)],
    vec![(4,0),(3,1),(2,2),(0,4)],
    vec![(5,0),(4,1),(3,2),(2,3),(1,4),(0,5)],
    vec![(5,1), (4,2), (3,3), (2,4), (1,5)],
    vec![(5,2), (4,3), (3,4), (2,5)],
    vec![(5,3), (4,4), (3,5)],
    vec![(5,4), (4,5)],
    vec![(5,5)],
])
)]
#[case::goal_unreachable_error(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0)),
(0, 0),
(5, 6),
10,
Err(MDDError::GoalUnreachable),
)]
fn test_mdd(
    #[case] scenario: crate::cbs::low_level::Grid,
    #[case] start: (i32, i32),
    #[case] goal: (i32, i32),
    #[case] c: i32,
    #[case] expected: Result<Vec<Vec<(i32, i32)>>, MDDError>,
) {
    let agent = crate::cbs::high_level::Agent {
        id: "a".to_string(),
        start,
        goal,
    };
    let mdd = super::mdd(&agent, &scenario, c);
    match mdd {
        Ok(mdd) => {
            assert_eq!(mdd, expected.unwrap());
        }
        Err(_) => {
            assert!(expected.is_err());
        }
    }
}

#[rstest]
#[case::simple(
    vec![
        vec![(0,0)],
        vec![(1,0),(0,1)],
        vec![(1,1)],
    ],
    vec![
        vec![(1,1)],
        vec![(1,0),(0,1)],
        vec![(0,0)],
    ],
    vec![
        vec![((0,0),(1,1))],
        vec![((1,0),(0,1)),((0,1),(1,0))],
        vec![((1,1),(0,0))],
    ],
)]
#[case::empty_joint_mdd(
    vec![
        vec![(0,0)],
        vec![(1,0)],
        vec![(1,1)],
    ],
    vec![
        vec![(1,1)],
        vec![(1,0)],
        vec![(0,0)],
    ],
    vec![
        vec![((0,0),(1,1))],
        vec![],
        vec![((1,1),(0,0))],
    ],
)]
#[case::different_lengths(
    vec![
        vec![(0,0)],
        vec![(1,0), (0,1)],
        vec![(1,1)],
        vec![(1,2)],
    ],
    vec![
        vec![(1,1)],
        vec![(1,0)],
        vec![(0,0)],
    ],
    vec![
        vec![((0,0),(1,1))],
        vec![((0,1), (1,0))],
        vec![((1,1),(0,0))],
        vec![((1,2),(0,0))],
    ],
)]
#[case::empty_mid_layers(
    vec![
        vec![(0,0)],
        vec![(0,1)],
        vec![(1,1)],
    ],
    vec![
        vec![(1,1)],
        vec![(0,1)],
        vec![(0,0)],
    ],
    vec![
        vec![((0,0),(1,1))],
        vec![],
        vec![((1,1),(0,0))],
    ],
)]
fn test_merge_mdds(
    #[case] mdd1: Vec<Vec<(i32, i32)>>,
    #[case] mdd2: Vec<Vec<(i32, i32)>>,
    #[case] expected: Vec<Vec<((i32, i32), (i32, i32))>>,
) {
    let merged = super::merge_mdds(&mdd1, &mdd2, (mdd1.len().max(mdd2.len()) - 1) as i32);
    assert_eq!(merged, expected);
}
