use super::*;
use crate::cbs::high_level::{Conflict, VertexConflict};
use rstest::rstest;

struct MockVertexConflict {
    agent1_idx: usize,
    agent2_idx: usize,
    location: (i32, i32),
    time: i32,
}

#[rstest]
#[case::single_conflict(
    vec![
        vec![(0, 0), (1, 0), (1, 1)],
        vec![(1, 1), (1, 0), (0, 0)],
    ],
    vec![
        MockVertexConflict {
            agent1_idx: 0,
            agent2_idx: 1,
            location: (1, 0),
            time: 1,
        },
    ],
    Some(0),
)]
#[case::one_semi_and_one_none(
    vec![
        vec![(0, 0), (1, 0)],
        vec![(1, 1), (1, 0), (0, 0)],
        vec![(2, 0), (1, 0), (1, 1)],
    ],
    vec![
        // non-cardinal
        MockVertexConflict {
            agent1_idx: 1,
            agent2_idx: 2,
            location: (1, 0),
            time: 1,
        },
        // semi-cardinal
        MockVertexConflict {
            agent1_idx: 0,
            agent2_idx: 1,
            location: (1, 0),
            time: 1,
        },
    ],
    Some(1),
)]
fn test_pick_conflict(
    #[case] paths: Vec<crate::cbs::high_level::Path>,
    #[case] conflicts: Vec<MockVertexConflict>,
    #[case] expected_idx: Option<usize>,
) {
    let grid =
        crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0));
    let mut path_map =
        HashMap::<&crate::cbs::high_level::Agent, crate::cbs::high_level::Path>::new();
    let mut agents = Vec::<crate::cbs::high_level::Agent>::new();
    for (i, path) in paths.iter().enumerate() {
        agents.push(crate::cbs::high_level::Agent {
            id: i.to_string(),
            start: path[0],
            goal: path[path.len() - 1],
        });
    }
    for (i, path) in paths.iter().enumerate() {
        path_map.insert(&agents[i], path.clone());
    }
    let processed_conflicts: Vec<Box<Conflict>> = conflicts
        .iter()
        .map(|c| {
            Box::new(Conflict::Vertex(VertexConflict {
                agent1: &agents[c.agent1_idx],
                agent2: &agents[c.agent2_idx],
                location: c.location,
                time: c.time,
            }))
        })
        .collect();
    let selected_conflict = pick_conflict(&grid, &path_map, &processed_conflicts);
    match expected_idx {
        Some(expected_idx) => match *processed_conflicts[expected_idx].clone() {
            Conflict::Vertex(c) => {
                let actual_c = match *selected_conflict.unwrap() {
                    Conflict::Vertex(c) => c,
                    _ => panic!("Expected vertex conflict"),
                };
                assert_eq!(actual_c, c);
            }
            Conflict::Edge(c) => {
                let actual_c = match *selected_conflict.unwrap() {
                    Conflict::Edge(c) => c,
                    _ => panic!("Expected edge conflict"),
                };
                assert_eq!(actual_c, c);
            }
        },
        None => {
            assert!(selected_conflict.is_none());
        }
    }
}
