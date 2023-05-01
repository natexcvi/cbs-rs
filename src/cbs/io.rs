use regex::Regex;
use std::fs::File;

use super::{
    high_level::Path,
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
}
