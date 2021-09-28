use clap;
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavHeader;

use crossbeam_channel::unbounded;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
use threadpool::ThreadPool;

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

    let verbose = matches.is_present("verbose");
    let connection_strings: Vec<String> = matches
        .values_of("connect")
        .unwrap()
        .map(Into::into)
        .collect();

    let (sender, receiver) = unbounded::<(String, (MavHeader, MavMessage))>();
    let pool = ThreadPool::with_name("mavlink worker".to_owned(), num_cpus::get());
    println!("Connecting to: {:?}", &connection_strings);
    for connection_string in connection_strings {
        let sender = sender.clone();
        let receiver = receiver.clone();

        let mut vehicle: Box<
            dyn mavlink::MavConnection<mavlink::ardupilotmega::MavMessage> + Sync + Send,
        > = mavlink::connect(&connection_string).unwrap();
        vehicle.set_protocol_version(mavlink::MavlinkVersion::V2);
        let vehicle = Arc::new(vehicle);

        let vehicle_send = vehicle.clone();
        let connection_string_send = connection_string.clone();
        pool.execute(move || {
            let connection_string: String = connection_string.into();
            loop {
                match vehicle.recv() {
                    Ok((header, msg)) => {
                        if verbose {
                            println!("[{}] > {:#?}", &connection_string, msg);
                        }
                        sender.send((connection_string.clone(), (header, msg)));
                    }
                    Err(error) => {
                        match error {
                            mavlink::error::MessageReadError::Io(error) => {
                                if error.kind() == std::io::ErrorKind::WouldBlock {
                                    //no messages currently available to receive -- wait a while
                                    thread::sleep(Duration::from_secs(1));
                                    dbg!("block!");
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
                thread::sleep(Duration::from_millis(10));
            }
        });
        pool.execute(move || loop {
            let messages: Vec<_> = receiver.try_iter().collect();
            for (name, (header, message)) in messages {
                if name != connection_string_send {
                    if verbose {
                        println!("[{}] < {:#?}", connection_string_send, message);
                    }
                    vehicle_send.send(&header, &message);
                }
            }
            thread::sleep(Duration::from_millis(10));
        });
    }

    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
