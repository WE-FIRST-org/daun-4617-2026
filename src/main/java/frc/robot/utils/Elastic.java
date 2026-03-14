// Copyright (c) 2023-2026 Gold87 and other Elastic contributors
// This software can be modified and/or shared under the terms
// defined by the Elastic license:
// https://github.com/Gold872/elastic_dashboard/blob/main/LICENSE

package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

public final class Elastic {
  private static final StringTopic notificationTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
  private static final StringPublisher notificationPublisher =
      notificationTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
  private static final StringTopic selectedTabTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher =
      selectedTabTopic.publish(PubSubOption.keepDuplicates(true));
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable elasticTable = inst.getTable("Elastic");
  private static final ObjectMapper objectMapper = new ObjectMapper();

  /**
   * Represents the possible levels of notifications for the Elastic dashboard. These levels are
   * used to indicate the severity or type of notification.
   */
  public enum NotificationLevel {
    /** Informational Message */
    INFO,
    /** Warning message */
    WARNING,
    /** Error message */
    ERROR
  }

  /**
   * Sends an notification to the Elastic dashboard. The notification is serialized as a JSON string
   * before being published.
   *
   * @param notification the {@link Notification} object containing notification details
   */
  public static void sendNotification(Notification notification) {
    try {
      notificationPublisher.set(objectMapper.writeValueAsString(notification));
    } catch (JsonProcessingException e) {
      e.printStackTrace();
    }
  }

  /**
   * Selects the tab of the dashboard with the given name. If no tab matches the name, this will
   * have no effect on the widgets or tabs in view.
   *
   * <p>If the given name is a number, Elastic will select the tab whose index equals the number
   * provided.
   *
   * @param tabName the name of the tab to select
   */
  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }

  /**
   * Selects the tab of the dashboard at the given index. If this index is greater than or equal to
   * the number of tabs, this will have no effect.
   *
   * @param tabIndex the index of the tab to select.
   */
  public static void selectTab(int tabIndex) {
    selectTab(Integer.toString(tabIndex));
  }

  /**
   * Represents an notification object to be sent to the Elastic dashboard. This object holds
   * properties such as level, title, description, display time, and dimensions to control how the
   * notification is displayed on the dashboard.
   */
  public static class Notification {
    @JsonProperty("level")
    private NotificationLevel level;

    @JsonProperty("title")
    private String title;

    @JsonProperty("description")
    private String description;

    @JsonProperty("displayTime")
    private int displayTimeMillis;

    @JsonProperty("width")
    private double width;

    @JsonProperty("height")
    private double height;

    /**
     * Creates a new Notification with all default parameters. This constructor is intended to be
     * used with the chainable decorator methods
     *
     * <p>Title and description fields are empty.
     */
    public Notification() {
      this(NotificationLevel.INFO, "", "");
    }

    /**
     * Creates a new Notification with all properties specified.
     *
     * @param level the level of the notification (e.g., INFO, WARNING, ERROR)
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param displayTimeMillis the time in milliseconds for which the notification is displayed
     * @param width the width of the notification display area
     * @param height the height of the notification display area, inferred if below zero
     */
    public Notification(
        NotificationLevel level,
        String title,
        String description,
        int displayTimeMillis,
        double width,
        double height) {
      this.level = level;
      this.title = title;
      this.displayTimeMillis = displayTimeMillis;
      this.description = description;
      this.height = height;
      this.width = width;
    }

    /**
     * Creates a new Notification with default display time and dimensions.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     */
    public Notification(NotificationLevel level, String title, String description) {
      this(level, title, description, 3000, 350, -1);
    }

    /**
     * Creates a new Notification with a specified display time and default dimensions.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param displayTimeMillis the display time in milliseconds
     */
    public Notification(
        NotificationLevel level, String title, String description, int displayTimeMillis) {
      this(level, title, description, displayTimeMillis, 350, -1);
    }

    /**
     * Creates a new Notification with specified dimensions and default display time. If the height
     * is below zero, it is automatically inferred based on screen size.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param width the width of the notification display area
     * @param height the height of the notification display area, inferred if below zero
     */
    public Notification(
        NotificationLevel level, String title, String description, double width, double height) {
      this(level, title, description, 3000, width, height);
    }

    /**
     * Updates the level of this notification
     *
     * @param level the level to set the notification to
     */
    public void setLevel(NotificationLevel level) {
      this.level = level;
    }

    /**
     * @return the level of this notification
     */
    public NotificationLevel getLevel() {
      return level;
    }

    /**
     * Updates the title of this notification
     *
     * @param title the title to set the notification to
     */
    public void setTitle(String title) {
      this.title = title;
    }

    /**
     * Gets the title of this notification
     *
     * @return the title of this notification
     */
    public String getTitle() {
      return title;
    }

    /**
     * Updates the description of this notification
     *
     * @param description the description to set the notification to
     */
    public void setDescription(String description) {
      this.description = description;
    }

    public String getDescription() {
      return description;
    }

    /**
     * Updates the display time of the notification
     *
     * @param seconds the number of seconds to display the notification for
     */
    public void setDisplayTimeSeconds(double seconds) {
      setDisplayTimeMillis((int) Math.round(seconds * 1000));
    }

    /**
     * Updates the display time of the notification in milliseconds
     *
     * @param displayTimeMillis the number of milliseconds to display the notification for
     */
    public void setDisplayTimeMillis(int displayTimeMillis) {
      this.displayTimeMillis = displayTimeMillis;
    }

    /**
     * Gets the display time of the notification in milliseconds
     *
     * @return the number of milliseconds the notification is displayed for
     */
    public int getDisplayTimeMillis() {
      return displayTimeMillis;
    }

    /**
     * Updates the width of the notification
     *
     * @param width the width to set the notification to
     */
    public void setWidth(double width) {
      this.width = width;
    }

    /**
     * Gets the width of the notification
     *
     * @return the width of the notification
     */
    public double getWidth() {
      return width;
    }

    /**
     * Updates the height of the notification
     *
     * <p>If the height is set to -1, the height will be determined automatically by the dashboard
     *
     * @param height the height to set the notification to
     */
    public void setHeight(double height) {
      this.height = height;
    }

    /**
     * Gets the height of the notification
     *
     * @return the height of the notification
     */
    public double getHeight() {
      return height;
    }

    /**
     * Modifies the notification's level and returns itself to allow for method chaining
     *
     * @param level the level to set the notification to
     * @return the current notification
     */
    public Notification withLevel(NotificationLevel level) {
      this.level = level;
      return this;
    }

    /**
     * Modifies the notification's title and returns itself to allow for method chaining
     *
     * @param title the title to set the notification to
     * @return the current notification
     */
    public Notification withTitle(String title) {
      setTitle(title);
      return this;
    }

    /**
     * Modifies the notification's description and returns itself to allow for method chaining
     *
     * @param description the description to set the notification to
     * @return the current notification
     */
    public Notification withDescription(String description) {
      setDescription(description);
      return this;
    }

    /**
     * Modifies the notification's display time and returns itself to allow for method chaining
     *
     * @param seconds the number of seconds to display the notification for
     * @return the current notification
     */
    public Notification withDisplaySeconds(double seconds) {
      return withDisplayMilliseconds((int) Math.round(seconds * 1000));
    }

    /**
     * Modifies the notification's display time and returns itself to allow for method chaining
     *
     * @param displayTimeMillis the number of milliseconds to display the notification for
     * @return the current notification
     */
    public Notification withDisplayMilliseconds(int displayTimeMillis) {
      setDisplayTimeMillis(displayTimeMillis);
      return this;
    }

    /**
     * Modifies the notification's width and returns itself to allow for method chaining
     *
     * @param width the width to set the notification to
     * @return the current notification
     */
    public Notification withWidth(double width) {
      setWidth(width);
      return this;
    }

    /**
     * Modifies the notification's height and returns itself to allow for method chaining
     *
     * @param height the height to set the notification to
     * @return the current notification
     */
    public Notification withHeight(double height) {
      setHeight(height);
      return this;
    }

    /**
     * Modifies the notification's height and returns itself to allow for method chaining
     *
     * <p>This will set the height to -1 to have it automatically determined by the dashboard
     *
     * @return the current notification
     */
    public Notification withAutomaticHeight() {
      setHeight(-1);
      return this;
    }

    /**
     * Modifies the notification to disable the auto dismiss behavior
     *
     * <p>This sets the display time to 0 milliseconds
     *
     * <p>The auto dismiss behavior can be re-enabled by setting the display time to a number
     * greater than 0
     *
     * @return the current notification
     */
    public Notification withNoAutoDismiss() {
      setDisplayTimeMillis(0);
      return this;
    }
  }

  // ── Data Publishing Methods (SmartDashboard replacement) ─────────────────────

  /**
   * Publishes a number value to the Elastic dashboard
   *
   * @param key the key name for the value
   * @param value the numeric value to publish
   */
  public static void putNumber(String key, double value) {
    elasticTable.getEntry(key).setDouble(value);
  }

  /**
   * Gets a number value from the Elastic dashboard, returns default if not found
   *
   * @param key the key name for the value
   * @param defaultValue the default value to return if key doesn't exist
   * @return the numeric value
   */
  public static double getNumber(String key, double defaultValue) {
    return elasticTable.getEntry(key).getDouble(defaultValue);
  }

  /**
   * Publishes a string value to the Elastic dashboard
   *
   * @param key the key name for the value
   * @param value the string value to publish
   */
  public static void putString(String key, String value) {
    elasticTable.getEntry(key).setString(value);
  }

  /**
   * Gets a string value from the Elastic dashboard, returns default if not found
   *
   * @param key the key name for the value
   * @param defaultValue the default value to return if key doesn't exist
   * @return the string value
   */
  public static String getString(String key, String defaultValue) {
    return elasticTable.getEntry(key).getString(defaultValue);
  }

  /**
   * Publishes a boolean value to the Elastic dashboard
   *
   * @param key the key name for the value
   * @param value the boolean value to publish
   */
  public static void putBoolean(String key, boolean value) {
    elasticTable.getEntry(key).setBoolean(value);
  }

  /**
   * Gets a boolean value from the Elastic dashboard, returns default if not found
   *
   * @param key the key name for the value
   * @param defaultValue the default value to return if key doesn't exist
   * @return the boolean value
   */
  public static boolean getBoolean(String key, boolean defaultValue) {
    return elasticTable.getEntry(key).getBoolean(defaultValue);
  }

  /**
   * Publishes a Sendable object to the Elastic dashboard
   * For complex objects like Field2d, this will serialize them to a NetworkTable structure
   *
   * @param key the key name for the value
   * @param data the Sendable object to publish
   */
  public static void putData(String key, Object data) {
    // For Field2d and other Sendable objects, we'll use the built-in NetworkTable publishing
    // This creates a subtable for the Sendable object
    NetworkTable subTable = elasticTable.getSubTable(key);
    if (data != null) {
      // Use the object's toString() method for basic serialization
      // More sophisticated serialization can be added as needed
      subTable.getEntry(".type").setString(data.getClass().getSimpleName());
      subTable.getEntry("value").setString(data.toString());
    }
  }

  // ── SendableChooser Replacement ─────────────────────────────────────────

  /**
   * A simple replacement for SendableChooser that works with Elastic dashboard
   */
  public static class ElasticChooser<T> {
    private final String key;
    private final Map<String, T> options = new HashMap<>();
    private String defaultOption = "";
    private String selectedOption = "";

    public ElasticChooser(String key) {
      this.key = key;
      // Publish the available options
      publishOptions();
    }

    /**
     * Adds an option to the chooser
     *
     * @param name the display name for the option
     * @param object the object associated with this option
     */
    public void addOption(String name, T object) {
      options.put(name, object);
      publishOptions();
    }

    /**
     * Sets the default option
     *
     * @param name the display name for the default option
     * @param object the object associated with this option
     */
    public void setDefaultOption(String name, T object) {
      defaultOption = name;
      selectedOption = name;
      options.put(name, object);
      publishOptions();
      publishSelection();
    }

    /**
     * Gets the currently selected object
     *
     * @return the selected object, or default if none selected
     */
    public T getSelected() {
      // Try to get selection from NetworkTable first
      String networkSelection = elasticTable.getEntry(key + "/selected").getString("");
      if (!networkSelection.isEmpty() && options.containsKey(networkSelection)) {
        return options.get(networkSelection);
      }
      // Fall back to default option
      return options.getOrDefault(defaultOption, null);
    }

    private void publishOptions() {
      List<String> optionNames = new ArrayList<>(options.keySet());
      elasticTable.getEntry(key + "/options").setStringArray(optionNames.toArray(new String[0]));
    }

    private void publishSelection() {
      elasticTable.getEntry(key + "/selected").setString(selectedOption);
    }
  }
}
