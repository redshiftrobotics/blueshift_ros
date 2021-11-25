export interface Notification {
	id: number;
	level: 'error' | 'info' | 'info-square' | 'success' | 'warning' | 'warning-alt';
	title: string;
	subtitle: string;
	caption?: string;
	type: 'toast' | 'permanent';
}
