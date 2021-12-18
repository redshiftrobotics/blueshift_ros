interface ROSMessageBase {
}

export interface geometry_msgs_Linear extends ROSMessageBase {
    x: number;
    y: number;
    z: number;
}

function geometry_msgs_Linear_Factory(): geometry_msgs_Linear {
    return {
        x: 0,
        y: 0,
        z: 0
    };
}

export interface geometry_msgs_Angular extends ROSMessageBase {
    x: number;
    y: number;
    z: number;
}

function geometry_msgs_Angular_Factory(): geometry_msgs_Angular {
    return {
        x: 0,
        y: 0,
        z: 0
    };
}

export interface geometry_msgs_Twist extends ROSMessageBase {
    linear: geometry_msgs_Linear;
    angular: geometry_msgs_Angular;
}

function geometry_msgs_Twist_Factory(): geometry_msgs_Twist {
    return {
        linear: geometry_msgs_Linear_Factory(),
        angular: geometry_msgs_Angular_Factory()
    };
}

export type ROSMessageStrings = "geometry_msgs/Linear" | "geometry_msgs/Angular" | "geometry_msgs/Twist";

export type ROSMessagesTypeTSDefinitions = {
    "geometry_msgs/Linear": geometry_msgs_Linear,
    "geometry_msgs/Angular": geometry_msgs_Angular,
    "geometry_msgs/Twist": geometry_msgs_Twist
}

// https://fettblog.eu/typescript-type-maps/
// https://blog.rsuter.com/how-to-instantiate-a-generic-type-in-typescript/
export type ROSMessage<T extends ROSMessageStrings> =
    T extends keyof ROSMessagesTypeTSDefinitions ? ROSMessagesTypeTSDefinitions[T] :
    ROSMessageBase;

export let ROSMessageFactories = {
    "geometry_msgs/Linear": geometry_msgs_Linear_Factory,
    "geometry_msgs/Angular": geometry_msgs_Angular_Factory,
    "geometry_msgs/Twist": geometry_msgs_Twist_Factory
}
