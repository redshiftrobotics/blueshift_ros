import { writable } from 'svelte/store';

export interface Notification {
	id?: number;
	level: 'error' | 'info' | 'info-square' | 'success' | 'warning' | 'warning-alt';
	title: string;
	subtitle: string;
	caption?: string;
	type: 'toast' | 'permanent' | 'permanent-no-toast';
}

function createNotificationManager(toastDisplayTime = 2500) {
	const { subscribe, set, update } = writable([]);

	let currId = 0;

	return {
		subscribe,
		addNotification: (notification: Notification) => {
			notification.id = ++currId;
			update((notifications) =>[...notifications, notification]);
			setTimeout(() => {
				if (notification.type === 'toast') {
					update((notifications) => notifications.filter((n) => n.id !== notification.id));
				} else if (notification.type === 'permanent') {
					update((notifications) => notifications.filter((n) => n.id !== notification.id));
					notification.type = 'permanent-no-toast';
					update((notifications) =>[...notifications, notification]);
				}
			}, toastDisplayTime);
		},
		removeNotification: (id: number) => update((n) => n.filter((n) => n.id !== id))
	};
}

export const notificationManager = createNotificationManager(5000);
