use regex::Regex;
use std::{fs::File, io::Read};

use super::{
    high_level::{Agent, Path},
    low_level::{Grid, LocationTime},
};

impl TryFrom<String> for Grid {
    fn try_from(value: String) -> Result<Self, Self::Error> {
        let map_regex = Regex::new(
            r"type octile\r?\nheight (\d+)\r?\nwidth (\d+)\r?\nmap\r?\n((?:[.G@OTSW]*\r?\n)*)",
        )
        .unwrap();
        let map_match = map_regex.captures(&value).ok_or("Invalid map file")?;

        let height = map_match[1]
            .parse::<i32>()
            .or(Result::Err("height is not a number"))?;
        let width = map_match[2]
            .parse::<i32>()
            .or(Result::Err("width is not a number"))?;
        let mut obstacles = Vec::new();
        for (i, line) in map_match[3].trim().split('\n').enumerate() {
            let line_vec: Vec<char> = line.trim().chars().collect();
            for (j, c) in line_vec.iter().enumerate() {
                if !matches!(c, '.' | 'G' | '@' | 'O' | 'T' | 'S' | 'W') {
                    return Err(format!("Invalid character: {}", c));
                }
                if c == &'@' {
                    obstacles.push(LocationTime {
                        location: (i as i32, j as i32),
                        time: -1,
                    });
                }
            }
        }
        Ok(Grid::new(width, height, obstacles, (0, 0)))
    }

    type Error = String;
}

impl TryInto<String> for Grid {
    fn try_into(self) -> Result<String, Self::Error> {
        let mut map = String::new();
        for i in 0..self.height {
            for j in 0..self.width {
                if self.obstacles.contains(&LocationTime {
                    location: (i, j),
                    time: -1,
                }) {
                    map.push('@');
                } else {
                    map.push('.');
                }
            }
            map.push('\n');
        }
        Ok(format!(
            "type octile\nheight {}\nwidth {}\nmap\n{}",
            self.height, self.width, map
        ))
    }

    type Error = String;
}

impl TryFrom<String> for Agent {
    type Error = String;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        todo!()
    }
}

pub fn load_scenario_file(scen_file: &str) -> Result<Vec<Agent>, String> {
    let agents_regex = Regex::new(r"version \d+(?:\.\d+)?\r?\n((?:\d+\t(?:.+)\t\d+\t\d+\t\d+\t\d+\t\d+\t\d+\t[\d.]+(?:\r?\n)?)*)").unwrap();
    let mut scen_file = File::open(scen_file).map_err(|e| e.to_string())?;
    let mut scen_content = String::new();
    scen_file
        .read_to_string(&mut scen_content)
        .map_err(|e| e.to_string())?;
    let scen_match = agents_regex
        .captures(&scen_content)
        .ok_or("Invalid scenario file")?;
    let agents_regex =
        Regex::new(r"(\d+)\t(.+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t([\d.]+)").unwrap();
    let mut agents: Vec<Agent> = Vec::new();
    for (i, caps) in agents_regex.captures_iter(&scen_match[1]).enumerate() {
        let x_start = caps[5].parse::<i32>().or(Err("start x not a number"))? + 1;
        let y_start = caps[6].parse::<i32>().or(Err("start y not a number"))? + 1;
        let x_goal = caps[7].parse::<i32>().or(Err("goal x not a number"))? + 1;
        let y_goal = caps[8].parse::<i32>().or(Err("goal y not a number"))? + 1;
        agents.push(Agent {
            id: i.to_string(),
            start: (x_start, y_start),
            goal: (x_goal, y_goal),
        });
    }
    Ok(agents)
}

#[cfg(test)]
mod tests {
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
}
