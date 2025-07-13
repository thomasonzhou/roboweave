import { type Node, type NodeProps, Position } from "@xyflow/react";
import { nanoid } from "nanoid";
import { memo, useMemo, useState } from "react";

import CustomHandle from "~/modules/flow-builder/components/handles/custom-handle";
import { type BaseNodeData, BuilderNode, type RegisterNodeMetadata } from "~/modules/nodes/types";
import { getNodeDetail } from "~/modules/nodes/utils";

import { cn } from "~@/utils/cn";

export interface WeaveMonitoringNodeData extends BaseNodeData {
    label?: string;
    features?: string[];
}

const NODE_TYPE = BuilderNode.WEAVE_MONITORING;

type WeaveMonitoringNodeProps = NodeProps<Node<WeaveMonitoringNodeData, typeof NODE_TYPE>>;

export function WeaveMonitoringNode({ data, selected }: WeaveMonitoringNodeProps) {
    const meta = useMemo(() => getNodeDetail(NODE_TYPE), []);

    const [targetHandleId] = useState<string>(nanoid());

    return (
        <>
            <div
                data-selected={selected}
                className="flex items-center border border-stone-300 rounded-xl bg-white px-4 py-3 shadow-sm transition data-[selected=true]:(border-orange-500 ring-1 ring-orange-500/50) min-w-52"
            >
                <div className="rounded-lg p-2 mr-3 bg-indigo-500">
                    <div className={cn(meta.icon, "size-4 text-white")} />
                </div>

                <div className="flex-1">
                    <div className="font-medium text-stone-800 text-sm">
                        {data.label || meta.title}
                    </div>
                    <div className="text-xs text-stone-500 mt-1">
                        Real-time observability • Performance tracking
                    </div>
                </div>
            </div>

            <CustomHandle
                type="target"
                id={targetHandleId}
                position={Position.Left}
                isConnectable={1}
            />
        </>
    );
}

export const metadata: RegisterNodeMetadata<WeaveMonitoringNodeData> = {
    type: NODE_TYPE,
    node: memo(WeaveMonitoringNode),
    detail: {
        icon: "i-mynaui:chart-bar",
        title: "Weave Monitoring",
        description: "Real-time observability and performance tracking",
    },
    defaultData: {
        label: "Weave Monitoring",
        features: ["Real-time observability", "Performance tracking"],
    },
}; 