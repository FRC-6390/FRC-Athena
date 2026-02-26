//! An example of a CLI that publishes to topics using a GenericPublisher.

use std::{collections::HashMap, io::stdin};

use nt_client::{data::r#type::DataType, publish::GenericPublisher, Client, ClientHandle};
use tokio::{select, sync::{broadcast, mpsc}};
use tracing::Level;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_max_level(Level::INFO)
        .init();

    let (cancel_send, mut cancel_recv) = broadcast::channel(1);

    let client = Client::new(Default::default());

    select! {
        // use exit here to forcibly stop the program without waiting for user input
        _ = cancel_recv.recv() => std::process::exit(0),
        res = client.connect_setup(|client| setup(client, cancel_send.clone())) => {
            let _ = cancel_send.send(());
            if let Err(err) = res {
                eprintln!("{err}");
            }
        },
    };
}

fn setup(client: &Client, cancel_send: broadcast::Sender<()>) {
    let mut cancel_recv = cancel_send.subscribe();

    // create a map of unknown-type publishers
    // this prevents publishers from unpublishing
    let mut publishers = HashMap::new();

    // client handle in order to create topics
    let handle = client.handle().clone();

    let sub_topic = handle.topic("/tmp");
    tokio::spawn(async move {
        // subscribe to a topic to be able to publish
        // this is a bug within NetworkTables, see https://github.com/wpilibsuite/allwpilib/issues/7680
        let sub_task = tokio::spawn(async move {
            let mut sub = sub_topic.subscribe(Default::default()).await.unwrap();
            loop {
                let _ = sub.recv().await;
            };
        });

        let (stdin_send, mut stdin_recv) = mpsc::channel(1);

        // reads the next line in stdin
        // spawn blocking, since read_line blocks the main thread
        let stdin_task = tokio::task::spawn_blocking(move || {
            loop {
                let mut command = String::new();
                stdin().read_line(&mut command).unwrap();
                if command.trim() == "quit" { break; };
                let _ = stdin_send.blocking_send(command.trim().to_string());
            };
        });

        // handles user input
        let command_task = tokio::spawn(async move {
            loop {
                // read a command from the stdin
                let Some(command) = stdin_recv.recv().await else { break; };
                let mut segments = command.splitn(2, " ");
                let Some(command) = segments.next() else {
                    eprintln!("malformed command");
                    continue;
                };
                let args = segments.next().unwrap_or("");

                match command {
                    // display some helpful information
                    "help" => {
                        println!("NT 4.1 Publisher CLI");
                        println!("Commands:");
                        println!("- publish <type> <topic>");
                        println!("  publishes to <topic> with type <type>");
                        println!("  possible types: string, boolean, int");
                        println!();
                        println!("- set <id> <value>");
                        println!("  publish to <id> <value>");
                        println!();
                        println!("- unpublish <id>");
                        println!("  unpublishes <id>");
                    }
                    // publish a topic to the server
                    "publish" => match publish_command(args, &handle, &mut publishers).await {
                        Ok(id) => println!("publishing with id {id}"),
                        Err(CommandError(err)) => eprintln!("{err}"),
                    }
                    // publish to a topic
                    "set" => match set_command(args, &mut publishers).await {
                        Ok(()) => println!("successfully set topic"),
                        Err(CommandError(err)) => eprintln!("{err}"),
                    }
                    // unpublish
                    "unpublish" => match unpublish_command(args, &mut publishers).await {
                        Ok(()) => println!("successfully unpublished"),
                        Err(CommandError(err)) => eprintln!("{err}"),
                    }
                    _ => eprintln!("unknown command {command}"),
                };
            };
        });

        select! {
            _ = cancel_recv.recv() => {},
            _ = sub_task => {},
            _ = stdin_task => {},
            res = command_task => {
                if let Err(err) = res {
                    eprintln!("{err}");
                }
            },
        };
        let _ = cancel_send.send(());
    });
}

// publish <type> <topic>
async fn publish_command(
    args: &str,
    handle: &ClientHandle,
    publishers: &mut HashMap<i32, GenericPublisher>,
) -> Result<i32, CommandError> {
    let mut args = args.split_whitespace();

    // the first arg will be the type
    let r#type = args.next().ok_or(CommandError("missing type arg".to_string()))?;
    // the rest will be the topic, joined by spaces
    let topic_name = args.collect::<Vec<&str>>().join(" ");
    if topic_name.is_empty() { return Err(CommandError("missing topic arg".to_string())); };

    // support strings, booleans, and integers
    let r#type = match r#type {
        "string" => DataType::String,
        "boolean" => DataType::Boolean,
        "int" => DataType::Int,
        _ => return Err(CommandError(format!("unknown type {type}"))),
    };

    let topic = handle.topic(topic_name);
    let publisher = topic.generic_publish(r#type, Default::default()).await?;
    let id = publisher.id();
    publishers.insert(id, publisher);
    Ok(id)
}

// set <id> <value>
async fn set_command(
    args: &str,
    publishers: &mut HashMap<i32, GenericPublisher>
) -> Result<(), CommandError> {
    // split the command args by whitespace
    let mut args = args.split_whitespace();

    // the first arg will be the id
    let id = args.next()
        .ok_or(CommandError("missing id arg".to_string()))?
        .parse::<i32>()?;
    // the rest will be the value, joined by spaces
    let value = args.collect::<Vec<&str>>().join(" ");
    if value.is_empty() { return Err(CommandError("missing topic arg".to_string())); };

    let publisher = publishers.get(&id).ok_or(CommandError("no publisher found".to_string()))?;

    // parse value as either a string, boolean, or integer and publish that value to the server
    match publisher.data_type() {
        DataType::String => publisher.set(value).await?,
        DataType::Boolean => publisher.set(value.parse::<bool>()?).await?,
        DataType::Int => publisher.set(value.parse::<i64>()?).await?,
        r#type => return Err(CommandError(format!("unsupported data type {type:?}"))),
    };

    Ok(())
}

// unpublish <id>
async fn unpublish_command(
    args: &str,
    publishers: &mut HashMap<i32, GenericPublisher>
) -> Result<(), CommandError> {
    // split the command args by whitespace
    let mut args = args.split_whitespace();

    // the first arg will be the id
    let id = args.next()
        .ok_or(CommandError("missing id arg".to_string()))?
        .parse::<i32>()?;

    // removing from the map causes the publisher to be owned by this function,
    // which will be dropped and unpublished by the end of this block
    publishers.remove(&id).ok_or(CommandError("publisher not found".to_string()))?;

    Ok(())
}

struct CommandError(String);

impl<T: ToString> From<T> for CommandError {
    fn from(value: T) -> Self {
        Self(value.to_string())
    }
}

