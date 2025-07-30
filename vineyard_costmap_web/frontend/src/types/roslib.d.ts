declare module 'roslib' {
  export default class ROSLIB {
    static Ros: new (options: {
      url: string;
      transportLibrary?: string;
      transportOptions?: any;
    }) => Ros;
    
    static Topic: new (options: {
      ros: Ros;
      name: string;
      messageType: string;
    }) => Topic;
    
    static Service: new (options: {
      ros: Ros;
      name: string;
      serviceType: string;
    }) => Service;
    
    static ServiceRequest: new (values?: any) => ServiceRequest;
    
    static Message: new (values?: any) => Message;
  }

  export interface Ros {
    connect(url: string): void;
    close(): void;
    on(event: string, callback: (error?: any) => void): void;
    isConnected: boolean;
  }

  export interface Topic {
    publish(message: Message): void;
    subscribe(callback: (message: any) => void): void;
    unsubscribe(): void;
    advertise(): void;
    unadvertise(): void;
  }

  export interface Service {
    callService(request: ServiceRequest, callback: (response: any) => void, failedCallback?: (error: any) => void): void;
  }

  export interface ServiceRequest {
    [key: string]: any;
  }

  export interface Message {
    [key: string]: any;
  }
}
