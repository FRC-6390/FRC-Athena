//! Collection of topics that can be used to subscribe to multiple topics at once.

use std::fmt::Debug;

use crate::{data::SubscriptionOptions, error::ConnectionClosedError, subscribe::Subscriber, ClientHandle};

use super::Topic;

/// Represents a collection of topics.
///
/// This is used to subscribe to multiple topics at once.
///
/// # Examples
/// ```
/// use nt_client::Client;
///
/// # tokio_test::block_on(async {
///     let client = Client::new(Default::default());
///
///     let topics = client.topics(vec![
///         "/topic".to_owned(),
///         "/nested/topic".to_owned(),
///         "/deeply/nested/topic".to_owned(),
///     ]);
///     tokio::spawn(async move {
///         let subscriber = topics.subscribe(Default::default()).await;
///
///         // do something with subscriber...
///     });
///
///     client.connect().await
/// # });
/// ```
#[derive(Debug, Clone)]
pub struct TopicCollection {
    names: Vec<String>,
    handle: ClientHandle,
}

impl IntoIterator for TopicCollection {
    type Item = Topic;
    type IntoIter = IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        IntoIter::new(self)
    }
}

impl PartialEq for TopicCollection {
    fn eq(&self, other: &Self) -> bool {
        self.names == other.names
    }
}

impl Eq for TopicCollection { }

impl TopicCollection {
    pub(crate) fn new(
        names: Vec<String>,
        handle: ClientHandle,
    ) -> Self {
        Self { names, handle }
    }

    /// Returns a slice of topic names this collection contains.
    pub fn names(&self) -> &Vec<String> {
        &self.names
    }

    /// Returns a mutable slice of topic names this collection contains.
    pub fn names_mut(&mut self) -> &mut Vec<String> {
        &mut self.names
    }

    /// Subscribes to this collection of topics.
    ///
    /// This method does not require the [`Client`] websocket connection to be made.
    ///
    /// [`Client`]: crate::Client
    pub async fn subscribe(&self, options: SubscriptionOptions) -> Result<Subscriber, ConnectionClosedError> {
        Subscriber::new(self.names.clone(), options, self.handle.announced_topics.clone(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }
}

/// Iterator that iterates over [`Topic`]s in a [`TopicCollection`].
///
/// This is obtained by the [`TopicCollection::into_iter`] method.
pub struct IntoIter {
    collection: TopicCollection,
}

impl Iterator for IntoIter {
    type Item = Topic;

    fn next(&mut self) -> Option<Self::Item> {
        let collection = &mut self.collection;
        if collection.names.is_empty() { return None; };

        Some(Topic::new(
            collection.names.remove(0),
            collection.handle.clone(),
        ))
    }
}

impl IntoIter {
    pub(self) fn new(collection: TopicCollection) -> Self {
        IntoIter {
            collection,
        }
    }
}

