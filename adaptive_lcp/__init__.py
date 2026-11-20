def classFactory(iface):
    from .plugin import AdaptiveLCPPlugin
    return AdaptiveLCPPlugin(iface)
