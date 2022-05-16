import { writable } from 'svelte/store';

/**
 * @param id the id of the notification, this is set by the notification manager
 * @param level the log level of the notification
 * @param title the title of the notification
 * @param subtitle the subtitle of the notification
 * @param caption the caption of the notification (this does not show up in toast notification)
 * @param type the type of notification, either toast or permanent
 *
 * `toast` is a notification that disappears after a set amount of time
 *
 * `permanent` is a toast that goes to the notification panel after a set amount of time
 *
 * `permanent-no-toast` is a notification that is only displated in the notification panel
 */
export interface Notification {
	id?: number;
	level: 'error' | 'info' | 'info-square' | 'success' | 'warning' | 'warning-alt';
	title: string;
	subtitle: string;
	caption?: string;
	type: 'toast' | 'permanent' | 'permanent-no-toast';
}

/**
 * Creates a notification manager store for displaying notifications
 * @param toastDisplayTime the amount of time in milliseconds that a toast notification will be displayed
 * @returns `subscribe`, `addNotification`, and `removeNotification`
 *
 * `subscribe` is a "normal" svelte store subscribe functoin
 * `addNotification` adds a notification to the notification list
 * `removeNotification` removes a notification from the notification list
 *
 * @remarks `toast` notifications are displayed by the _OverlayToastNotififcations_ component. `permanent` notifications are displayed by the _Header_ component.
 */
function createNotificationManager(toastDisplayTime = 2500) {
	const { subscribe, set, update } = writable([]);

	let currId = 0;

	return {
		subscribe,
		addNotification: (notification: Notification) => {
			notification.id = ++currId;
			update((notifications) => [...notifications, notification]);
			setTimeout(() => {
				if (notification.type === 'toast') {
					update((notifications) => notifications.filter((n) => n.id !== notification.id));
				} else if (notification.type === 'permanent') {
					update((notifications) => {
						// if it is still in the array (the user didn't manually remove it)
						// then remove it, and add it back with the new type
						if (notifications.some((n) => n.id === notification.id)) {
							// remove it
							let notifications_without_notification = notifications.filter(
								(n) => n.id !== notification.id
							);

							// add it back with the new type and sort the array to make sure everythign is in the right order
							notification.type = 'permanent-no-toast';
							return [...notifications_without_notification, notification].sort(
								(a, b) => a.id - b.id
							);
						} else {
							return notifications;
						}
					});
				}
			}, toastDisplayTime);
		},
		removeNotification: (id: number) => update((n) => n.filter((n) => n.id !== id))
	};
}

export const notificationManager = createNotificationManager(3500);
