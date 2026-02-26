//! Named data channels.
//!
//! Topics have a fixed data type and can be subscribed and published to.

use std::{collections::{HashMap, VecDeque}, fmt::{Debug, Display}, time::Duration};

use crate::{data::{r#type::{DataType, NetworkTableData}, Announce, Properties, SubscriptionOptions, Unannounce}, error::ConnectionClosedError, publish::{GenericPublisher, NewPublisherError, Publisher}, subscribe::Subscriber, ClientHandle};

pub mod collection;

/// Creates a [`TopicPath`] containing the segments.
///
/// `path!` allows for the easy creation of [`TopicPath`]s without having to deal with creating
/// [`VecDeque`]s.
/// Its usage is extremely similar to [`std::vec!`].
///
/// # Examples
/// ```
/// use std::collections::VecDeque;
/// use nt_client::{topic::TopicPath, path};
///
/// let mut vec_deque = VecDeque::new();
/// vec_deque.push_back("my".to_owned());
/// vec_deque.push_back("path".to_owned());
/// let path = TopicPath::new(vec_deque);
/// 
/// assert_eq!(path!["my", "path"], path);
/// ```
#[macro_export]
macro_rules! path {
    () => {
        $crate::topic::TopicPath::default();
    };

    ($($segment: literal),+ $(,)?) => {{
        let mut segments = std::collections::VecDeque::new();
        $(
            segments.push_back($segment.to_string());
        )*
        $crate::topic::TopicPath::new(segments)
    }};
}

/// Represents a `NetworkTables` topic.
///
/// This differs from an [`AnnouncedTopic`], as that is a **server created topic**, while this is a
/// **client created topic**.
///
/// # Examples
/// ```no_run
/// use nt_client::{Client, path};
///
/// # tokio_test::block_on(async {
/// let client = Client::new(Default::default());
///
/// client.connect_setup(|client| {
///     // get a topic named `/my/topic` using the path! macro
///     let mut topic = client.topic(path!["my", "topic"]);
///     tokio::spawn(async move {
///         // subscribe to `/mytopic`
///         // note that this immediately unsubscribes and doesn't actually do anything
///         topic.subscribe(Default::default()).await;
///
///         // mutate the topic, changing its name to `/mytopic`
///         // this is useful when you don't have access to a Client and need to dynamically
///         // "create" topics
///         //
///         // to see a better example of this, look at the `generic_pub` example
///         // the subscriber created previously, assuming it isn't immediately dropped,
///         // will still be subscribing to `/mytopic`
///         *topic.name_mut() = "/othertopic".to_owned();
///
///         // subscribe to `/othertopic`
///         // note that this immediately unsubscribes and doesn't actually do anything
///         topic.subscribe(Default::default()).await;
///     });
/// }).await.unwrap();
/// # });
/// ```
#[derive(Debug, Clone)]
pub struct Topic {
    name: String,
    handle: ClientHandle,
}

impl PartialEq for Topic {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

impl Eq for Topic { }

impl Topic {
    pub(super) fn new(
        name: String,
        handle: ClientHandle
    ) -> Self {
        Self { name, handle }
    }

    /// Returns a reference to the name this topic has.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns a mutable reference to the name this topic has.
    pub fn name_mut(&mut self) -> &mut String {
        &mut self.name
    }

    /// Publishes to this topic with the data type `T`.
    ///
    /// For a generic-free version, see [`generic_publish`][`Self::generic_publish`].
    ///
    /// # Note
    /// This method requires the [`Client`] websocket connection to already be made. Calling this
    /// method wihout already connecting the [`Client`] will cause it to hang forever.
    ///
    /// # Errors
    /// Returns an error if a publisher could not be made to the server.
    ///
    /// [`Client`]: crate::Client
    pub async fn publish<T: NetworkTableData>(&self, properties: Properties) -> Result<Publisher<T>, NewPublisherError> {
        Publisher::new(self.name.clone(), properties, self.handle.time(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }

    /// Publishes to this topic with the data type `T`, not waiting for an nnounce message from the
    /// server. This is meant as a workaround to [issue #7680][https://github.com/wpilibsuite/allwpilib/issues/7680].
    ///
    /// Using this method does not guarantees that the topic has a matching type, nor does it
    /// guarantee that the publisher was even able to be made. Use at your own risk!
    ///
    /// # Note
    /// This method requires the [`Client`] websocket connection to already be made. Calling this
    /// method wihout already connecting the [`Client`] will cause it to hang forever.
    ///
    /// # Errors
    /// Returns an error if the `NetworkTables` connection was closed.
    ///
    /// [`Client`]: crate::Client
    #[cfg(feature = "publish_bypass")]
    pub async fn publish_bypass<T: NetworkTableData>(&self, properties: Properties) -> Result<Publisher<T>, ConnectionClosedError> {
        Publisher::new_bypass(self.name.clone(), properties, self.handle.time(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }

    /// Publishes to this topic with some data type.
    ///
    /// This behaves differently from [`publish`][`Self::publish`], as that has a generic type and
    /// guarantees through type-safety that the client is publishing values that have the same type
    /// the server does for that topic. Extra care must be taken to ensure no type mismatches
    /// occur.
    ///
    /// # Note
    /// This method requires the [`Client`] websocket connection to already be made. Calling this
    /// method wihout already connecting the [`Client`] will cause it to hang forever.
    ///
    /// # Errors
    /// Returns an error if a publisher could not be made to the server.
    ///
    /// [`Client`]: crate::Client
    pub async fn generic_publish(&self, r#type: DataType, properties: Properties) -> Result<GenericPublisher, NewPublisherError> {
        GenericPublisher::new(self.name.clone(), properties, r#type, self.handle.time(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }

    /// Publishes to this topic with some data type, not waiting for an nnounce message from the
    /// server. This is meant as a workaround to [issue #7680][https://github.com/wpilibsuite/allwpilib/issues/7680].
    ///
    /// Using this method does not guarantees that the topic has a matching type, nor does it
    /// guarantee that the publisher was even able to be made. Use at your own risk!
    ///
    /// This behaves differently from [`publish_bypass`][`Self::publish_bypass`], as that has a generic type and
    /// guarantees through type-safety that the client is publishing values that have the same type
    /// the server does for that topic. Extra care must be taken to ensure no type mismatches
    /// occur.
    ///
    /// # Note
    /// This method requires the [`Client`] websocket connection to already be made. Calling this
    /// method wihout already connecting the [`Client`] will cause it to hang forever.
    ///
    /// # Errors
    /// Returns an error if the `NetworkTables` connection was closed.
    ///
    /// [`Client`]: crate::Client
    #[cfg(feature = "publish_bypass")]
    pub async fn generic_publish_bypass(&self, r#type: DataType, properties: Properties) -> Result<GenericPublisher, ConnectionClosedError> {
        GenericPublisher::new_bypass(self.name.clone(), properties, r#type, self.handle.time(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }

    /// Subscribes to this topic.
    ///
    /// This method does not require the [`Client`] websocket connection to be made.
    ///
    /// [`Client`]: crate::Client
    pub async fn subscribe(&self, options: SubscriptionOptions) -> Result<Subscriber, ConnectionClosedError> {
        Subscriber::new(vec![self.name.clone()], options, self.handle.announced_topics.clone(), self.handle.send_ws.0.clone(), self.handle.recv_ws.0.subscribe()).await
    }
}

/// A topic that has been announced by the `NetworkTables` server.
///
/// Topics will only be announced when there is a subscriber subscribing to it.
///
/// This differs from a [`Topic`], as that is a **client created topic**, while this is a
/// **server created topic**.
#[derive(Debug, Clone, PartialEq)]
pub struct AnnouncedTopic {
    name: String,
    id: i32,
    r#type: DataType,
    pub(crate) properties: Properties,
    value: Option<rmpv::Value>,
    last_updated: Option<Duration>,
}

impl AnnouncedTopic {
    /// Returns the name of this topic.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the id of this topic.
    ///
    /// This id is guaranteed to be unique.
    pub fn id(&self) -> i32 {
        self.id
    }

    /// Returns the data type of this topic.
    pub fn r#type(&self) -> &DataType {
        &self.r#type
    }

    /// Returns the properties of this topic.
    pub fn properties(&self) -> &Properties {
        &self.properties
    }

    /// Returns the current value of this topic.
    ///
    /// This value will not be present if the `cached` property is `false`, regardless of if this
    /// topic has been published to.
    pub fn value(&self) -> Option<&rmpv::Value> {
        self.value.as_ref()
    }

    /// Returns when this topic was last updated as a duration of time since the server started.
    ///
    /// This value will not be present if it has never been published to.
    pub fn last_updated(&self) -> Option<&Duration> {
        self.last_updated.as_ref()
    }

    pub(crate) fn update(&mut self, when: Duration) {
        self.last_updated = Some(when);
    }

    pub(crate) fn update_value(&mut self, value: rmpv::Value) {
        self.value = Some(value);
    }

    /// Returns whether the given names and subscription options match this topic.
    pub fn matches(&self, names: &[String], options: &SubscriptionOptions) -> bool {
        names.iter()
            .any(|name| &self.name == name || (options.prefix.is_some_and(|flag| flag) && self.name.starts_with(name)))
    }
}

impl From<&Announce> for AnnouncedTopic {
    fn from(value: &Announce) -> Self {
        Self {
            name: value.name.clone(),
            id: value.id,
            r#type: value.r#type,
            properties: value.properties.clone(),
            value: None,
            last_updated: None,
        }
    }
}

/// Represents a list of all server-announced topics.
#[derive(Default, Debug, Clone, PartialEq)]
pub struct AnnouncedTopics {
    topics: HashMap<i32, AnnouncedTopic>,
    name_to_id: HashMap<String, i32>,
}

impl AnnouncedTopics {
    /// Creates a new, empty list of announced topics.
    pub fn new() -> Self {
        Default::default()
    }

    pub(crate) fn insert(&mut self, announce: &Announce) {
        self.topics.insert(announce.id, announce.into());
        self.name_to_id.insert(announce.name.clone(), announce.id);
    }

    pub(crate) fn remove(&mut self, unannounce: &Unannounce) {
        self.topics.remove(&unannounce.id);
        self.name_to_id.remove(&unannounce.name);
    }

    /// Gets a topic from its id.
    pub fn get_from_id(&self, id: i32) -> Option<&AnnouncedTopic> {
        self.topics.get(&id)
    }

    /// Gets a mutable topic from its id.
    pub fn get_mut_from_id(&mut self, id: i32) -> Option<&mut AnnouncedTopic> {
        self.topics.get_mut(&id)
    }

    /// Gets a topic from its name.
    pub fn get_from_name(&self, name: &str) -> Option<&AnnouncedTopic> {
        self.name_to_id.get(name).and_then(|id| self.topics.get(id))
    }

    /// Gets a mutable topic from its name.
    pub fn get_mut_from_name(&mut self, name: &str) -> Option<&mut AnnouncedTopic> {
        self.name_to_id.get(name).and_then(|id| self.topics.get_mut(id))
    }

    /// Gets a topic id from its name.
    pub fn get_id(&self, name: &str) -> Option<i32> {
        self.name_to_id.get(name).copied()
    }

    /// An iterator visiting all id and topic values in arbitrary order.
    pub fn id_values(&self) -> std::collections::hash_map::Values<'_, i32, AnnouncedTopic> {
        self.topics.values()
    }
}

/// Represents a slash (`/`) deliminated path.
///
/// This is especially useful when trying to parse nested data, such as from Shuffleboard
/// (found in `/Shuffleboard/...`).
///
/// This can be thought of as a wrapper for a [`VecDeque`], only providing trait impls to convert
/// to/from a [`String`].
///
/// # Note
/// The [`Display`] impl will always contain a leading slash, but not a trailing one,
/// regardless of if the path was parsed from a [`String`] containing either a leading or trailing
/// slash.
///
/// # Warning
/// In cases where slashes are present in segment names, turning to and from a [`String`] is
/// **NOT** guaranteed to preserve segment names.
///
/// ```
/// use nt_client::{topic::TopicPath, path};
///
/// let path = path!["///weird//", "na//mes//"];
///
/// assert_ne!(<String as Into<TopicPath>>::into(path.to_string()), path);
/// ```
///
/// In the above example, `.to_string()` is converting the path to `////weird///na//mes//`.
/// When turning this back into a `TopicPath`, it recognizes the following segments (with
/// trailing and leading slashes removed):
///
/// **/** / **/weird** / **/** / **na** / **/mes** /
///
/// # Examples
/// ```
/// use nt_client::{topic::TopicPath, path};
///
/// // automatically inserts a leading slash
/// assert_eq!(path!["my", "topic"].to_string(), "/my/topic");
///
/// // slashes in the segment names are preserved
/// assert_eq!(path!["some", "/data"].to_string(), "/some//data");
///
/// assert_eq!(<&str as Into<TopicPath>>::into("/path/to/data"), path!["path", "to", "data"]);
///
/// assert_eq!(<&str as Into<TopicPath>>::into("//some///weird/path/"), path!["/some", "/", "weird", "path"]);
/// ```
/// Getting a topic:
/// ```no_run
/// use nt_client::{Client, path};
///
/// # tokio_test::block_on(async {
/// let client = Client::new(Default::default());
///
/// let topic = client.topic(path!["nested", "data"]);
///
/// // do something with `topic`
///
/// client.connect().await;
/// # });
/// ```
/// Parsing topic name:
/// ```no_run
/// use nt_client::{topic::TopicPath, data::SubscriptionOptions, subscribe::ReceivedMessage, Client};
///
/// # tokio_test::block_on(async {
/// let client = Client::new(Default::default());
///
/// client.connect_setup(setup).await;
/// # });
///
/// fn setup(client: &Client) {
///     let sub_topic = client.topic("/Root/");
///     tokio::spawn(async move {
///         let mut sub = sub_topic.subscribe(SubscriptionOptions {
///             topics_only: Some(true),
///             prefix: Some(true),
///             ..Default::default()
///         }).await.unwrap();
///
///         while let Ok(ReceivedMessage::Announced(topic)) = sub.recv().await {
///             let path: TopicPath = topic.name().into();
///
///             // do something with `path`
///         }
///     });
/// }
/// ```
#[derive(Default, Debug, Clone, PartialEq, Eq, Hash)]
pub struct TopicPath {
    /// The segments contained in the path.
    pub segments: VecDeque<String>,
}

impl TopicPath {
    /// The delimiter to use when converting from a [`String`].
    pub const DELIMITER: char = '/';

    /// Creates a new `TopicPath` with segments.
    pub fn new(segments: VecDeque<String>) -> Self {
        Self { segments }
    }
}

impl From<VecDeque<String>> for TopicPath {
    fn from(value: VecDeque<String>) -> Self {
        Self { segments: value }
    }
}

impl Display for TopicPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let full_path = self.segments.iter().fold(String::new(), |prev, curr| prev + "/" + curr);
        f.write_str(&full_path)
    }
}

impl From<&str> for TopicPath {
    fn from(value: &str) -> Self {
        value.to_string().into()
    }
}

impl From<String> for TopicPath {
    fn from(value: String) -> Self {
        let str = value.strip_prefix(Self::DELIMITER).unwrap_or(&value);
        let str = str
            .strip_suffix(Self::DELIMITER)
            .map(|str| str.to_owned())
            .unwrap_or_else(|| str.to_owned());

        str.chars().fold((VecDeque::<String>::new(), true), |(mut parts, prev_is_delimiter), char| {
            if prev_is_delimiter {
                parts.push_back(String::from(char));
                (parts, false)
            } else {
                let is_delimiter = char == Self::DELIMITER;
                if !is_delimiter { parts.back_mut().unwrap().push(char); };
                (parts, is_delimiter)
            }
        }).0.into()
    }
}

