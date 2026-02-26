//! Topic subscribers.
//!
//! Subscribers receive data value updates to a topic.
//!
//! # Examples
//!
//! ```no_run
//! use nt_client::{subscribe::ReceivedMessage, data::r#type::NetworkTableData, Client};
//!
//! # tokio_test::block_on(async {
//! let client = Client::new(Default::default());
//!
//! client.connect_setup(setup).await.unwrap();
//! # });
//!
//! fn setup(client: &Client) {
//!     // prints updates to the `/counter` topic to the stdout
//!     let counter_topic = client.topic("/counter");
//!     tokio::spawn(async move {
//!         // subscribes to the `/counter`
//!         let mut subscriber = counter_topic.subscribe(Default::default()).await.unwrap();
//!
//!         loop {
//!             match subscriber.recv().await {
//!                 Ok(ReceivedMessage::Updated((_topic, value))) => {
//!                     // get the updated value as an `i32`
//!                     let number = i32::from_value(&value).unwrap();
//!                     println!("counter updated to {number}");
//!                 },
//!                 Ok(ReceivedMessage::Announced(topic)) => println!("announced topic: {topic:?}"),
//!                 Ok(ReceivedMessage::Unannounced { name, .. }) => println!("unannounced topic: {name}"),
//!                 Ok(ReceivedMessage::UpdateProperties(topic)) => println!("topic {} had its properties updated: {:?}", topic.name(), topic.properties()),
//!                 Err(err) => {
//!                     eprintln!("got error: {err:?}");
//!                     break;
//!                 },
//!             }
//!         }
//!     });
//! }

use std::{collections::{HashMap, HashSet}, fmt::Debug, sync::Arc};

use futures_util::future::join_all;
use tokio::sync::{broadcast, RwLock};
use tracing::warn;

use crate::{data::{BinaryData, ClientboundData, ClientboundTextData, PropertiesData, ServerboundMessage, ServerboundTextData, Subscribe, SubscriptionOptions, Unsubscribe}, error::ConnectionClosedError, recv_until_async, topic::{AnnouncedTopic, AnnouncedTopics}, NTClientReceiver, NTServerSender};

/// A `NetworkTables` subscriber that subscribes to a [`Topic`].
///
/// Subscribers receive topic announcements, value updates, topic unannouncements, and topic
/// property change messages.
///
/// This will automatically get unsubscribed whenever this goes out of scope.
///
/// [`Topic`]: crate::topic::Topic
pub struct Subscriber {
    topics: Vec<String>,
    id: i32,
    options: SubscriptionOptions,
    topic_ids: Arc<RwLock<HashSet<i32>>>,
    announced_topics: Arc<RwLock<AnnouncedTopics>>,

    ws_sender: NTServerSender,
    ws_recv: NTClientReceiver,
}

impl Debug for Subscriber {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Subscriber")
            .field("topics", &self.topics)
            .field("id", &self.id)
            .field("options", &self.options)
            .field("topic_ids", &self.topic_ids)
            .finish()
    }
}

impl PartialEq for Subscriber {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for Subscriber { }

impl Subscriber {
    pub(super) async fn new(
        topics: Vec<String>,
        options: SubscriptionOptions,
        announced_topics: Arc<RwLock<AnnouncedTopics>>,
        ws_sender: NTServerSender,
        ws_recv: NTClientReceiver,
    ) -> Result<Self, ConnectionClosedError> {
        let id = rand::random();

        let topic_ids = {
            let announced_topics = announced_topics.read().await;
            announced_topics.id_values()
                .filter(|topic| topic.matches(&topics, &options))
                .map(|topic| topic.id())
                .collect()
        };

        let sub_message = ServerboundTextData::Subscribe(Subscribe { topics: topics.clone(), subuid: id, options: options.clone() });
        ws_sender.send(ServerboundMessage::Text(sub_message).into()).map_err(|_| ConnectionClosedError)?;

        Ok(Self {
            topics,
            id,
            options,
            topic_ids: Arc::new(RwLock::new(topic_ids)),
            announced_topics,
            ws_sender,
            ws_recv
        })
    }

    /// Returns all topics that this subscriber is subscribed to.
    pub async fn topics(&self) -> HashMap<i32, AnnouncedTopic> {
        let topic_ids = self.topic_ids.clone();
        let topic_ids = topic_ids.read().await;
        let mapped_futures = topic_ids.iter()
            .map(|id| {
                let announced_topics = self.announced_topics.clone();
                async move {
                    (*id, announced_topics.read().await.get_from_id(*id).expect("topic exists").clone())
                }
            });
        join_all(mapped_futures).await.into_iter().collect()
    }

    /// Receives the next value for this subscriber.
    ///
    /// Topics that have already been announced will not be received by this method. To view
    /// all topics that are being subscribed to, use the [`topics`][`Self::topics`] method.
    ///
    /// # Errors
    /// Returns an error if something goes wrong when receiving messages from the client.
    pub async fn recv(&mut self) -> Result<ReceivedMessage, broadcast::error::RecvError> {
        recv_until_async(&mut self.ws_recv, |data| {
            let topic_ids = self.topic_ids.clone();
            let announced_topics = self.announced_topics.clone();
            let topics = &self.topics;
            let options = &self.options;
            async move {
                match *data {
                    ClientboundData::Binary(BinaryData { id, ref timestamp, ref data, .. }) => {
                        let contains = {
                            topic_ids.read().await.contains(&id)
                        };
                        if !contains { return None; };
                        let announced_topic = {
                            let mut topics = announced_topics.write().await;
                            let Some(topic) = topics.get_mut_from_id(id) else {
                                warn!("unknown topic with id {id}");
                                return None;
                            };

                            if topic.last_updated().is_some_and(|last_timestamp| last_timestamp > timestamp) { return None; };
                            topic.update(*timestamp);
                            if topic.properties().cached.is_some_and(|cached| cached) {
                                topic.update_value(data.clone());
                            };

                            topic.clone()
                        };
                        Some(ReceivedMessage::Updated((announced_topic, data.clone())))
                    },
                    ClientboundData::Text(ClientboundTextData::Announce(ref announce)) => {
                        let matches = announced_topics.read().await.get_from_id(announce.id).is_some_and(|topic| topic.matches(topics, options));
                        if matches {
                            topic_ids.write().await.insert(announce.id);
                            Some(ReceivedMessage::Announced(announce.into()))
                        } else { None }
                    },
                    ClientboundData::Text(ClientboundTextData::Unannounce(ref unannounce)) => {
                        topic_ids.write().await.remove(&unannounce.id).then(|| {
                            ReceivedMessage::Unannounced { name: unannounce.name.clone(), id: unannounce.id }
                        })
                    },
                    ClientboundData::Text(ClientboundTextData::Properties(PropertiesData { ref name, .. })) => {
                        let (contains, id) = {
                            let Some(id) = announced_topics.read().await.get_id(name) else {
                                warn!("unknown topic {name}");
                                return None;
                            };
                            (topic_ids.read().await.contains(&id), id)
                        };
                        if !contains { return None; };

                        let topics = announced_topics.read().await;
                        let topic = topics.get_from_id(id).expect("topic exists").clone();
                        Some(ReceivedMessage::UpdateProperties(topic))
                    },
                }
            }
        }).await
    }
}

impl Drop for Subscriber {
    fn drop(&mut self) {
        let unsub_message = ServerboundTextData::Unsubscribe(Unsubscribe { subuid: self.id });
        // if the receiver is dropped, the ws connection is closed
        let _ = self.ws_sender.send(ServerboundMessage::Text(unsub_message).into());
    }
}

/// Messages that can received from a subscriber.
#[derive(Debug, Clone, PartialEq)]
pub enum ReceivedMessage {
    /// A topic that matches the subscription options and subscribed topics was announced.
    ///
    /// This will always be received before any updates for that topic are sent.
    Announced(AnnouncedTopic),
    /// An subscribed topic was updated.
    ///
    /// Subscribed topics are any topics that were [`Announced`][`ReceivedMessage::Announced`].
    /// Only the most recent updated value is sent.
    Updated((AnnouncedTopic, rmpv::Value)),
    /// An announced topic had its properties updated.
    UpdateProperties(AnnouncedTopic),
    /// An announced topic was unannounced.
    Unannounced {
        /// The name of the topic that was unannounced.
        name: String,
        /// The id of the topic that was unannounced.
        id: i32,
    },
}

