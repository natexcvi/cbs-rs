use std::collections::HashMap;

use crate::cbs::high_level::Path;
use crate::cbs::mdd::mdd;
use crate::cbs::{
    high_level::{Agent, Conflict},
    low_level::Grid,
};

#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum ConflictCardinality {
    Cardinal,
    SemiCardinal,
    NonCardinal,
}

fn cardinality(
    scenario: &Grid,
    paths: &HashMap<&Agent, Path>,
    conflict: &Conflict,
) -> ConflictCardinality {
    let agent1: &Agent;
    let agent2: &Agent;
    let time: i32;
    match conflict {
        Conflict::Vertex(c) => {
            agent1 = c.agent1;
            agent2 = c.agent2;
            time = c.time;
        }
        Conflict::Edge(c) => {
            agent1 = c.agent1;
            agent2 = c.agent2;
            time = c.time;
        }
    }
    let c1 = paths[agent1].len();
    let c2 = paths[agent2].len();
    let agent1_mdd = mdd(agent1, scenario, c1 as i32).unwrap();
    let agent2_mdd = mdd(agent2, scenario, c2 as i32).unwrap();
    if agent1_mdd[time.clone() as usize].len() == 1
        && agent2_mdd
            .get(time.clone() as usize)
            .unwrap_or(agent2_mdd.last().unwrap())
            .len()
            == 1
    {
        return ConflictCardinality::Cardinal;
    } else if agent1_mdd[time.clone() as usize].len() == 1
        || agent2_mdd
            .get(time.clone() as usize)
            .unwrap_or(agent2_mdd.last().unwrap())
            .len()
            == 1
    {
        return ConflictCardinality::SemiCardinal;
    } else {
        return ConflictCardinality::NonCardinal;
    }
}

pub fn pick_conflict<'a>(
    scenario: &Grid,
    paths: &HashMap<&Agent, Path>,
    conflicts: &Vec<Box<Conflict<'a>>>,
) -> Option<Box<Conflict<'a>>> {
    let min_conflict = conflicts
        .iter()
        .min_by(|a, b| cardinality(scenario, paths, a).cmp(&cardinality(scenario, paths, b)));
    match min_conflict {
        Some(c) => Some(c.clone()),
        None => None,
    }
}

#[cfg(test)]
mod tests;
