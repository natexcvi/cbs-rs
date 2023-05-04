mod cbs;

use cbs::io::paths_to_string;
use cbs::{CBSInstance, CBS};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    map_file: String,

    #[arg(short, long)]
    agents_file: String,

    #[arg(short, long)]
    output_file: Option<String>,
}

fn main() {
    env_logger::init();
    let args = Args::parse();
    let cbs_instance = CBSInstance::from_files(&args.map_file, &args.agents_file)
        .expect("should be valid scenario files");
    let mut cbs = CBS::new(cbs_instance, None);
    match cbs.solve() {
        Ok(paths) => {
            println!("{}", paths_to_string(&paths));
        }
        Err(e) => panic!("CBS Error: {:?}", e),
    }
}
