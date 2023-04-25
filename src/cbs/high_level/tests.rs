use super::*;
use rstest::*;

#[test]
fn test_high_level() {
    let agents = vec![
        Agent {
            id: "a".to_string(),
            start: (0, 0),
            goal: (9, 9),
        },
        Agent {
            id: "b".to_string(),
            start: (0, 1),
            goal: (9, 8),
        },
    ];
    let constraints = vec![
        Box::new(Constraint {
            agent: &agents[0],
            time: 0,
            location: (1, 0),
        }),
        Box::new(Constraint {
            agent: &agents[1],
            time: 0,
            location: (8, 0),
        }),
    ];
    let precomputed_paths = HashMap::<&Agent, Vec<(i32, i32)>>::new();
    let grid = Grid {
        width: 10,
        height: 10,
        obstacles: vec![],
        goal: (9, 9),
    };
    let ctn = ConflictTreeNode::new(
        agents.iter().collect(),
        constraints,
        precomputed_paths,
        &grid,
    );
    assert_eq!(ctn.conflicts.len(), 1);
    match ctn.expand() {
        Some(expanded) => {
            assert_eq!(expanded.len(), 2);
            assert_eq!(expanded[0].constraints.len(), 3);
            assert_eq!(expanded[1].constraints.len(), 3);
            assert_eq!(expanded[0].constraints[2].time, 17);
            assert_eq!(expanded[0].conflicts.len(), 0);
        }
        None => panic!("No expanded nodes"),
    }
}

#[rstest]
#[case::simple(vec![
    Agent {
        id: "a".to_string(),
        start: (0, 0),
        goal: (9, 9),
    },
    Agent {
        id: "b".to_string(),
        start: (0, 1),
        goal: (9, 8),
    },
], Grid {
    width: 10,
    height: 10,
    obstacles: vec![],
    goal: (9, 9),
}, vec![19, 17])]
#[case::opposite_corners(vec![
    Agent {
        id: "a".to_string(),
        start: (9, 9),
        goal: (0, 0),
    },
    Agent {
        id: "b".to_string(),
        start: (0, 0),
        goal: (9, 9),
    },
], Grid {
    width: 10,
    height: 10,
    obstacles: vec![],
    goal: (9, 9),
}, vec![19, 19])]
#[case::crowded(vec![
    Agent {
        id: "a".to_string(),
        start: (0, 0),
        goal: (1, 1),
    },
    Agent {
        id: "b".to_string(),
        start: (1, 0),
        goal: (0, 0),
    },
    Agent {
        id: "c".to_string(),
        start: (0, 1),
        goal: (1, 0),
    },
], Grid {
    width: 2,
    height: 2,
    obstacles: vec![],
    goal: (0, 0),
}, vec![3, 2, 3])]
#[case::crowded_with_obstacles(vec![
    Agent {
        id: "a".to_string(),
        start: (0, 0),
        goal: (2, 2),
    },
    Agent {
        id: "b".to_string(),
        start: (1, 0),
        goal: (0, 0),
    },
    Agent {
        id: "c".to_string(),
        start: (0, 1),
        goal: (1, 0),
    },
], Grid {
    width: 3,
    height: 3,
    obstacles: vec![LocationTime { location: (1,1), time: 2 }],
    goal: (0, 0),
}, vec![5, 2, 3])]
fn test_cbs(#[case] agents: Vec<Agent>, #[case] grid: Grid, #[case] exp_path_lengths: Vec<usize>) {
    let mut cbs = CBS::new(&grid, agents.iter().collect());
    match cbs.solve() {
        Ok(paths) => {
            assert_eq!(paths.len(), exp_path_lengths.len());
            for (agent, path) in paths.iter() {
                assert_eq!(
                    path.len(),
                    exp_path_lengths[agents.iter().position(|a| a == *agent).unwrap()]
                );
            }
        }
        Err(e) => panic!("Error: {:?}", e),
    }
}
