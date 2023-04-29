use super::*;
use rstest::rstest;

#[rstest]
#[case::simple(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
    }),
    vec![
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
#[case::opposite_corners(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
    }),
    vec![
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
#[case::crowded(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
    }),
    vec![
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
#[case::crowded_with_obstacles(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
    }),
    vec![
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
obstacles: vec![low_level::LocationTime { location: (1,1), time: 2 }],
goal: (0, 0),
}, vec![5, 2, 3])]
#[case::must_wait(
    Some(CBSOptimisationConfig {
        priotising_conflicts: true,
        bypassing_conflicts: true,
    }),
    vec![
Agent {
    id: "a".to_string(),
    start: (0, 1),
    goal: (2, 1),
},
Agent {
    id: "b".to_string(),
    start: (1, 0),
    goal: (1, 2),
},
], Grid {
width: 3,
height: 3,
obstacles: vec![low_level::LocationTime { location: (2,0), time: -1 }, low_level::LocationTime { location: (0,2), time: -1 }],
goal: (0, 0),
}, vec![3, 4])]
fn test_cbs(
    #[case] optimisation_config: Option<CBSOptimisationConfig>,
    #[case] agents: Vec<Agent>,
    #[case] grid: Grid,
    #[case] exp_path_lengths: Vec<usize>,
) {
    let mut cbs = CBS::new(&grid, agents.iter().collect(), optimisation_config);
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
