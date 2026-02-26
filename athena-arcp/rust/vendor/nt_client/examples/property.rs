//! An example of changing topic properties.

use std::{collections::HashMap, time::Duration};

use nt_client::{data::Properties, publish::UpdateProps, Client};
use serde_json::json;

#[tokio::main]
async fn main() {
    let client = Client::new(Default::default());

    client.connect_setup(setup).await.unwrap()
}

fn setup(client: &Client) {
    let topic = client.topic("/mytopic");
    tokio::spawn(async move {
        let mut extra_props = HashMap::new();
        extra_props.insert("custom".to_string(), json!("something"));

        // initial properties are:
        // - cached: true
        // - custom: `something`
        // everything else is unset
        let mut publisher = topic.publish::<String>(Properties { cached: Some(true), extra: extra_props, ..Default::default() }).await.unwrap();

        // after 1 second...
        tokio::time::sleep(Duration::from_secs(1)).await;

        let updated = UpdateProps::new()
            .set_persistent(true)
            .delete("custom".to_string());

        // update properties
        // updated properties are:
        // - cached: true
        // - persistent: true
        // everything else is unset (including the `custom` property)
        publisher.update_props(updated).await.unwrap();
    });
}

