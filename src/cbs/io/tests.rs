use super::*;
use rstest::rstest;
use std::io::Read;

#[rstest]
#[case::clean_24x24("tests/testdata/maps/clean_24x24.map", 24, 24, vec![])]
#[case::clean_24x24_crlf("tests/testdata/maps/clean_24x24_crlf.map", 24, 24, vec![])]
#[case::clean_24x24_crlf("tests/testdata/maps/5x3_with_conflict.map", 5, 3, vec![
    LocationTime { location: (3, 1), time: -1 },
])]
fn test_map_loading_saving(
    #[case] map_file_path: &str,
    #[case] height: i32,
    #[case] width: i32,
    #[case] obstacles: Vec<LocationTime>,
) {
    let mut map_file = File::open(map_file_path).unwrap();
    let mut map_file_content = String::new();
    map_file.read_to_string(&mut map_file_content).unwrap();
    let map = Grid::try_from(map_file_content).unwrap();
    assert_eq!(map.height, height);
    assert_eq!(map.width, width);
    assert_eq!(map.obstacles, obstacles);

    let map_file_content: String = map.try_into().unwrap();
    let map = Grid::try_from(map_file_content).unwrap();
    assert_eq!(map.height, height);
    assert_eq!(map.width, width);
    assert_eq!(map.obstacles, obstacles);
}

#[rstest]
#[case::simple_24x24_12_agents("tests/testdata/scenarios/24x24_12_agents.scen", vec![
    Agent { id: "0".to_string(), start: (12, 1), goal: (12, 12) },
    Agent { id: "1".to_string(), start: (1, 12), goal: (12, 23) },
    Agent { id: "2".to_string(), start: (11, 2), goal: (24, 13) },
    Agent { id: "3".to_string(), start: (10, 3), goal: (24, 14) },
    Agent { id: "4".to_string(), start: (9, 4), goal: (24, 15) },
    Agent { id: "5".to_string(), start: (8, 5), goal: (24, 16) },
    Agent { id: "6".to_string(), start: (7, 6), goal: (24, 17) },
    Agent { id: "7".to_string(), start: (6, 7), goal: (24, 18) },
    Agent { id: "8".to_string(), start: (5, 8), goal: (24, 19) },
    Agent { id: "9".to_string(), start: (4, 9), goal: (24, 20) },
    Agent { id: "10".to_string(), start: (3, 10), goal: (24, 21) },
    Agent { id: "11".to_string(), start: (2, 11), goal: (24, 22) }
])]
fn test_load_scenario_file(#[case] scen_file_path: &str, #[case] exp_agents: Vec<Agent>) {
    let agents = load_scenario_file(scen_file_path).unwrap();
    assert_eq!(agents, exp_agents);
}
