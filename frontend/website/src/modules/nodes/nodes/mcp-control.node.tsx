import { type Node, type NodeProps, Position } from "@xyflow/react";
import { nanoid } from "nanoid";
import { memo, useMemo, useState, useEffect } from "react";

import CustomHandle from "~/modules/flow-builder/components/handles/custom-handle";
import { type BaseNodeData, BuilderNode, type RegisterNodeMetadata } from "~/modules/nodes/types";
import { getNodeDetail } from "~/modules/nodes/utils";
import { useMCPRobot } from "~/hooks/use-mcp-robot";

import { cn } from "~@/utils/cn";

export interface MCPControlNodeData extends BaseNodeData {
    label?: string;
    serverUrl?: string;
    autoConnect?: boolean;
}

const NODE_TYPE = BuilderNode.MCP_PROTOCOL;

type MCPControlNodeProps = NodeProps<Node<MCPControlNodeData, typeof NODE_TYPE>>;

export function MCPControlNode({ data, selected, isConnectable }: MCPControlNodeProps) {
    const meta = useMemo(() => getNodeDetail(NODE_TYPE), []);
    const { connected, connecting, robotState, connect, disconnect } = useMCPRobot();

    const [targetHandleId] = useState<string>(nanoid());
    const [sourceHandleId] = useState<string>(nanoid());

    // Auto-connect if enabled
    useEffect(() => {
        if (data.autoConnect && !connected && !connecting) {
            connect();
        }
    }, [data.autoConnect, connected, connecting, connect]);

    const getStatusColor = () => {
        if (connecting) return "bg-amber-500";
        if (connected) return "bg-green-500";
        return "bg-red-500";
    };

    const getStatusText = () => {
        if (connecting) return "Connecting...";
        if (connected) return "Connected";
        return "Disconnected";
    };

    return (
        <>
            <div
                data-selected={selected}
                className="flex flex-col border border-stone-300 rounded-xl bg-white px-4 py-3 shadow-sm transition data-[selected=true]:(border-orange-500 ring-1 ring-orange-500/50) min-w-64"
            >
                {/* Header */}
                <div className="flex items-center mb-2">
                    <div className={cn("rounded-lg p-2 mr-3", getStatusColor())}>
                        <div className={cn(meta.icon, "size-4 text-white")} />
                    </div>

                    <div className="flex-1">
                        <div className="font-medium text-stone-800 text-sm">
                            {data.label || meta.title}
                        </div>
                        <div className="text-xs text-stone-500">
                            {getStatusText()}
                        </div>
                    </div>
                </div>

                {/* Robot State Display */}
                {connected && robotState && (
                    <div className="mt-2 p-2 bg-stone-50 rounded-lg">
                        <div className="text-xs font-medium text-stone-700 mb-1">Robot State</div>
                        <div className="text-xs text-stone-600 space-y-1">
                            <div>
                                Position: ({robotState.position.x.toFixed(2)}, {robotState.position.y.toFixed(2)}, {robotState.position.z.toFixed(2)})
                            </div>
                            <div>
                                Mode: {robotState.current_mode}
                            </div>
                        </div>
                    </div>
                )}

                {/* Connection Controls */}
                <div className="mt-2 flex gap-2">
                    {!connected && !connecting && (
                        <button 
                            className="px-2 py-1 text-xs bg-orange-500 text-white rounded hover:bg-orange-600 transition-colors"
                            onClick={connect}
                        >
                            Connect
                        </button>
                    )}
                    {connected && (
                        <button 
                            className="px-2 py-1 text-xs bg-red-500 text-white rounded hover:bg-red-600 transition-colors"
                            onClick={disconnect}
                        >
                            Disconnect
                        </button>
                    )}
                </div>
            </div>

            <CustomHandle
                type="target"
                id={targetHandleId}
                position={Position.Left}
                isConnectable={isConnectable}
            />
            <CustomHandle
                type="source"
                id={sourceHandleId}
                position={Position.Right}
                isConnectable={isConnectable}
            />
        </>
    );
}

export const metadata: RegisterNodeMetadata<MCPControlNodeData> = {
    type: NODE_TYPE,
    node: memo(MCPControlNode),
    detail: {
        icon: "i-mynaui:layers",
        title: "MCP Control",
        description: "Live Model Context Protocol connection for robot communication",
    },
    defaultData: {
        label: "MCP Control",
        serverUrl: "http://localhost:8080",
        autoConnect: true,
    },
};
