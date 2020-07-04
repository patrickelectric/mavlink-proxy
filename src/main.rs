use clap;

use std::sync::{Arc, RwLock};

use std::thread;
use std::time::Duration;

fn main() {
    let matches = clap::App::new(env!("CARGO_PKG_NAME"))
        .version(env!("CARGO_PKG_VERSION"))
        .about("MAVLink proxy!")
        .author(env!("CARGO_PKG_AUTHORS"))
        .arg(
            clap::Arg::with_name("connect")
                .short("c")
                .long("connect")
                .value_name("TYPE:<IP/SERIAL>:<PORT/BAUDRATE>")
                .help("Sets the mavlink connection string")
                .takes_value(true)
                .multiple(true)
                .default_value("udpin:0.0.0.0:14550"),
        )
        .arg(
            clap::Arg::with_name("verbose")
                .short("v")
                .long("verbose")
                .help("Be verbose")
                .takes_value(false),
        )
        .get_matches();

    let verbose = Arc::new(matches.is_present("verbose"));
    let connection_strings: Vec<_> = matches.values_of("connect").unwrap().collect();

    if *verbose {
        for index in 0..connection_strings.len() {
            println!(
                "Connection: {} with index {}.",
                connection_strings[index], index
            );
        }
    }

    let vehicles: Arc<
        Vec<
            Arc<
                RwLock<
                    Box<
                        dyn mavlink::MavConnection<mavlink::ardupilotmega::MavMessage>
                            + Sync
                            + Send,
                    >,
                >,
            >,
        >,
    > = Arc::new(
        connection_strings
            .iter()
            .map(|&connection_string| {
                Arc::new(RwLock::new(mavlink::connect(&connection_string).unwrap()))
            })
            .collect(),
    );

    let mut threads = vec![];
    for index in 0..vehicles.len() {
        threads.push(thread::spawn({
            let verbose = Arc::clone(&verbose);
            let vehicles = Arc::clone(&vehicles);
            move || {
                let vehicle = Arc::clone(&vehicles[index]);
                let vehicle = vehicle.read().unwrap();
                loop {
                    match vehicle.recv() {
                        Ok((header, msg)) => {
                            if *verbose {
                                println!("[{}] > {:#?}", index, msg);
                            }
                            for other_index in 0..vehicles.len() {
                                if index != other_index {
                                    if *verbose {
                                        println!("[{}] < {:#?}", other_index, msg);
                                    }
                                    let vehicle = vehicles[other_index].read().unwrap();
                                    let _ = vehicle.send(&header, &msg);
                                }
                            }
                        }
                        Err(error) => {
                            match error {
                                mavlink::error::MessageReadError::Io(error) => {
                                    if error.kind() == std::io::ErrorKind::WouldBlock {
                                        //no messages currently available to receive -- wait a while
                                        thread::sleep(Duration::from_secs(1));
                                        continue;
                                    }
                                }
                                _ => {
                                    println!("Got error: {:?}", error);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }));
    }

    for handle in threads {
        let _ = handle.join();
    }
}
