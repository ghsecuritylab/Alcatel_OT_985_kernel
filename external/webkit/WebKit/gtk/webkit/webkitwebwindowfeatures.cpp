

#include "config.h"

#include "WindowFeatures.h"
#include "webkitwebwindowfeatures.h"
#include "webkitprivate.h"


enum {
    PROP_0,

    PROP_X,
    PROP_Y,
    PROP_WIDTH,
    PROP_HEIGHT,
    PROP_TOOLBAR_VISIBLE,
    PROP_STATUSBAR_VISIBLE,
    PROP_SCROLLBAR_VISIBLE,
    PROP_MENUBAR_VISIBLE,
    PROP_LOCATIONBAR_VISIBLE,
    PROP_FULLSCREEN,
};

G_DEFINE_TYPE(WebKitWebWindowFeatures, webkit_web_window_features, G_TYPE_OBJECT)

struct _WebKitWebWindowFeaturesPrivate {
    gint x;
    gint y;
    gint width;
    gint height;

    gboolean toolbar_visible;
    gboolean statusbar_visible;
    gboolean scrollbar_visible;
    gboolean menubar_visible;
    gboolean locationbar_visible;

    gboolean fullscreen;
};

#define WEBKIT_WEB_WINDOW_FEATURES_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_WEB_WINDOW_FEATURES, WebKitWebWindowFeaturesPrivate))

static void webkit_web_window_features_set_property(GObject* object, guint prop_id, const GValue* value, GParamSpec* pspec);

static void webkit_web_window_features_get_property(GObject* object, guint prop_id, GValue* value, GParamSpec* pspec);

static void webkit_web_window_features_class_init(WebKitWebWindowFeaturesClass* klass)
{
    GObjectClass* gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->set_property = webkit_web_window_features_set_property;
    gobject_class->get_property = webkit_web_window_features_get_property;

    GParamFlags flags = (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT);

    webkit_init();

    /**
     * WebKitWebWindowFeatures:x:
     *
     * The starting x position of the window on the screen.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_X,
                                    g_param_spec_int(
                                    "x",
                                    "x",
                                    "The starting x position of the window on the screen.",
                                    -1,
                                    G_MAXINT,
                                    -1,
                                    flags));

    /**
     * WebKitWebWindowFeatures:y:
     *
     * The starting y position of the window on the screen.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_Y,
                                    g_param_spec_int(
                                    "y",
                                    "y",
                                    "The starting y position of the window on the screen.",
                                    -1,
                                    G_MAXINT,
                                    -1,
                                    flags));

    /**
     * WebKitWebWindowFeatures:width:
     *
     * The width of the window on the screen.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_WIDTH,
                                    g_param_spec_int(
                                    "width",
                                    "Width",
                                    "The width of the window on the screen.",
                                    -1,
                                    G_MAXINT,
                                    -1,
                                    flags));

    /**
     * WebKitWebWindowFeatures:height:
     *
     * The height of the window on the screen.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_HEIGHT,
                                    g_param_spec_int(
                                    "height",
                                    "Height",
                                    "The height of the window on the screen.",
                                    -1,
                                    G_MAXINT,
                                    -1,
                                    flags));

    /**
     * WebKitWebWindowFeatures:toolbar-visible:
     *
     * Controls whether the toolbar should be visible for the window.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_TOOLBAR_VISIBLE,
                                    g_param_spec_boolean(
                                    "toolbar-visible",
                                    "Toolbar Visible",
                                    "Controls whether the toolbar should be visible for the window.",
                                    TRUE,
                                    flags));

    /**
     * WebKitWebWindowFeatures:statusbar-visible:
     *
     * Controls whether the statusbar should be visible for the window.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_STATUSBAR_VISIBLE,
                                    g_param_spec_boolean(
                                    "statusbar-visible",
                                    "Statusbar Visible",
                                    "Controls whether the statusbar should be visible for the window.",
                                    TRUE,
                                    flags));

    /**
     * WebKitWebWindowFeatures:scrollbar-visible:
     *
     * Controls whether the scrollbars should be visible for the window.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_SCROLLBAR_VISIBLE,
                                    g_param_spec_boolean(
                                    "scrollbar-visible",
                                    "Scrollbar Visible",
                                    "Controls whether the scrollbars should be visible for the window.",
                                    TRUE,
                                    flags));

    /**
     * WebKitWebWindowFeatures:menubar-visible:
     *
     * Controls whether the menubar should be visible for the window.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_MENUBAR_VISIBLE,
                                    g_param_spec_boolean(
                                    "menubar-visible",
                                    "Menubar Visible",
                                    "Controls whether the menubar should be visible for the window.",
                                    TRUE,
                                    flags));

    /**
     * WebKitWebWindowFeatures:locationbar-visible:
     *
     * Controls whether the locationbar should be visible for the window.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_LOCATIONBAR_VISIBLE,
                                    g_param_spec_boolean(
                                    "locationbar-visible",
                                    "Locationbar Visible",
                                    "Controls whether the locationbar should be visible for the window.",
                                    TRUE,
                                    flags));

    /**
     * WebKitWebWindowFeatures:fullscreen:
     *
     * Controls whether window will be displayed fullscreen.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(gobject_class,
                                    PROP_FULLSCREEN,
                                    g_param_spec_boolean(
                                    "fullscreen",
                                    "Fullscreen",
                                    "Controls whether window will be displayed fullscreen.",
                                    FALSE,
                                    flags));


    g_type_class_add_private(klass, sizeof(WebKitWebWindowFeaturesPrivate));
}

static void webkit_web_window_features_init(WebKitWebWindowFeatures* web_window_features)
{
    web_window_features->priv = WEBKIT_WEB_WINDOW_FEATURES_GET_PRIVATE(web_window_features);
}

static void webkit_web_window_features_set_property(GObject* object, guint prop_id, const GValue* value, GParamSpec* pspec)
{
    WebKitWebWindowFeatures* web_window_features = WEBKIT_WEB_WINDOW_FEATURES(object);
    WebKitWebWindowFeaturesPrivate* priv = web_window_features->priv;

    switch(prop_id) {
    case PROP_X:
        priv->x = g_value_get_int(value);
        break;
    case PROP_Y:
        priv->y = g_value_get_int(value);
        break;
    case PROP_WIDTH:
        priv->width = g_value_get_int(value);
        break;
    case PROP_HEIGHT:
        priv->height = g_value_get_int(value);
        break;
    case PROP_TOOLBAR_VISIBLE:
        priv->toolbar_visible = g_value_get_boolean(value);
        break;
    case PROP_STATUSBAR_VISIBLE:
        priv->statusbar_visible = g_value_get_boolean(value);
        break;
    case PROP_SCROLLBAR_VISIBLE:
        priv->scrollbar_visible = g_value_get_boolean(value);
        break;
    case PROP_MENUBAR_VISIBLE:
        priv->menubar_visible = g_value_get_boolean(value);
        break;
    case PROP_LOCATIONBAR_VISIBLE:
        priv->locationbar_visible = g_value_get_boolean(value);
        break;
    case PROP_FULLSCREEN:
        priv->fullscreen = g_value_get_boolean(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void webkit_web_window_features_get_property(GObject* object, guint prop_id, GValue* value, GParamSpec* pspec)
{
    WebKitWebWindowFeatures* web_window_features = WEBKIT_WEB_WINDOW_FEATURES(object);
    WebKitWebWindowFeaturesPrivate* priv = web_window_features->priv;

    switch (prop_id) {
    case PROP_X:
        g_value_set_int(value, priv->x);
        break;
    case PROP_Y:
        g_value_set_int(value, priv->y);
        break;
    case PROP_WIDTH:
        g_value_set_int(value, priv->width);
        break;
    case PROP_HEIGHT:
        g_value_set_int(value, priv->height);
        break;
    case PROP_TOOLBAR_VISIBLE:
        g_value_set_boolean(value, priv->toolbar_visible);
        break;
    case PROP_STATUSBAR_VISIBLE:
        g_value_set_boolean(value, priv->statusbar_visible);
        break;
    case PROP_SCROLLBAR_VISIBLE:
        g_value_set_boolean(value, priv->scrollbar_visible);
        break;
    case PROP_MENUBAR_VISIBLE:
        g_value_set_boolean(value, priv->menubar_visible);
        break;
    case PROP_LOCATIONBAR_VISIBLE:
        g_value_set_boolean(value, priv->locationbar_visible);
        break;
    case PROP_FULLSCREEN:
        g_value_set_boolean(value, priv->fullscreen);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

WebKitWebWindowFeatures* webkit_web_window_features_new()
{
    return WEBKIT_WEB_WINDOW_FEATURES(g_object_new(WEBKIT_TYPE_WEB_WINDOW_FEATURES, NULL));
}

// for internal use only
WebKitWebWindowFeatures* webkit_web_window_features_new_from_core_features(const WebCore::WindowFeatures& features)
{
    WebKitWebWindowFeatures *webWindowFeatures = webkit_web_window_features_new();

    if(features.xSet)
        g_object_set(webWindowFeatures, "x", static_cast<int>(features.x), NULL);

    if(features.ySet)
        g_object_set(webWindowFeatures, "y", static_cast<int>(features.y), NULL);

    if(features.widthSet)
        g_object_set(webWindowFeatures, "width", static_cast<int>(features.width), NULL);

    if(features.heightSet)
        g_object_set(webWindowFeatures, "height", static_cast<int>(features.height), NULL);

    g_object_set(webWindowFeatures,
                 "toolbar-visible", features.toolBarVisible,
                 "statusbar-visible", features.statusBarVisible,
                 "scrollbar-visible", features.scrollbarsVisible,
                 "menubar-visible", features.menuBarVisible,
                 "locationbar-visible", features.locationBarVisible,
                 "fullscreen", features.fullscreen,
                 NULL);

    return webWindowFeatures;
}

gboolean webkit_web_window_features_equal(WebKitWebWindowFeatures* features1, WebKitWebWindowFeatures* features2)
{
    WebKitWebWindowFeaturesPrivate* priv1 = features1->priv;
    WebKitWebWindowFeaturesPrivate* priv2 = features2->priv;

    if((priv1->x == priv2->x) &&
       (priv1->y == priv2->y) &&
       (priv1->width == priv2->width) &&
       (priv1->height == priv2->height) &&
       (priv1->toolbar_visible == priv2->toolbar_visible) &&
       (priv1->statusbar_visible == priv2->statusbar_visible) &&
       (priv1->scrollbar_visible == priv2->scrollbar_visible) &&
       (priv1->menubar_visible == priv2->menubar_visible) &&
       (priv1->locationbar_visible == priv2->locationbar_visible) &&
       (priv1->fullscreen == priv2->fullscreen))
        return TRUE;
    return FALSE;
}
