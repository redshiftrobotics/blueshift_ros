import log from 'loglevel';
import { notificationManager } from './notification_manager';

var originalFactory = log.methodFactory;
log.methodFactory = function (methodName, logLevel, loggerName) {
    var rawMethod = originalFactory(methodName, logLevel, loggerName);

    return function (message) {
        switch (methodName) {
            case "trace":
                break;
            case "debug":
                break;
            case "info":
                break;
            case "warn":
                notificationManager.addNotification({
                    title: `JS Warning: ${message}`,
                    level: 'warning-alt',
                    subtitle: typeof loggerName !== 'undefined' ? loggerName.toString() : 'global logger',
                    caption: new Date().toLocaleString(),
                    type: 'permanent'
                });
                break;
            case "error":
                notificationManager.addNotification({
                    title: `JS Error: ${message}`,
                    level: 'error',
                    subtitle: typeof loggerName !== 'undefined' ? loggerName.toString() : 'global logger',
                    caption: new Date().toLocaleString(),
                    type: 'permanent'
                });
                break;
        }
        rawMethod(message);
    };
};
log.setLevel(log.getLevel());

log.enableAll();

export default log