declare module 'roslib' {
  namespace ROSLIB {
    interface RosOptions {
      url: string;
      transportLibrary?: string;
      transportOptions?: any;
    }

    interface TopicOptions {
      ros: Ros;
      name: string;
      messageType: string;
    }

    interface ServiceOptions {
      ros: Ros;
      name: string;
      serviceType: string;
    }

    interface ServiceRequest {
      [key: string]: any;
    }

    interface Message {
      [key: string]: any;
    }

    class Ros {
      constructor(options: RosOptions);
      connect(url: string): void;
      close(): void;
      on(event: string, callback: (error?: any) => void): void;
      isConnected: boolean;
    }

    class Topic {
      constructor(options: TopicOptions);
      publish(message: Message): void;
      subscribe(callback: (message: any) => void): void;
      unsubscribe(): void;
      advertise(): void;
      unadvertise(): void;
    }

    class Service {
      constructor(options: ServiceOptions);
      callService(request: ServiceRequest, callback: (response: any) => void, failedCallback?: (error: any) => void): void;
    }

    class ServiceRequest {
      constructor(values?: any);
      [key: string]: any;
    }

    class Message {
      constructor(values?: any);
      [key: string]: any;
    }
  }

  export = ROSLIB;
}
