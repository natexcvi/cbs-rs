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
mod tests;
