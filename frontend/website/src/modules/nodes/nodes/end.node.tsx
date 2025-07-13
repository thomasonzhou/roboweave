import { type Node, type NodeProps, Position } from "@xyflow/react";
import { nanoid } from "nanoid";
import { memo, useMemo, useState } from "react";

import CustomHandle from "~/modules/flow-builder/components/handles/custom-handle";
import { type BaseNodeData, BuilderNode, type RegisterNodeMetadata } from "~/modules/nodes/types";
import { getNodeDetail } from "~/modules/nodes/utils";

import { cn } from "~@/utils/cn";

export interface EndNodeData extends BaseNodeData {
    label?: string;
}

const NODE_TYPE = BuilderNode.END;

type EndNodeProps = NodeProps<Node<EndNodeData, typeof NODE_TYPE>>;

export function EndNode({ data, selected }: EndNodeProps) {
    const meta = useMemo(() => getNodeDetail(NODE_TYPE), []);

    const [sourceHandleId] = useState<string>(nanoid());

    return (
        <>
            <div
                data-selected={selected}
                data-deletable={false}
                className="flex items-center border border-dark-100 rounded-full bg-dark-300 px-4 py-2 shadow-sm transition data-[selected=true]:(border-teal-600 ring-1 ring-teal-600/50)"
            >
                <div className={cn(meta.icon, "size-4.5 shrink-0 mr-2 scale-130")} />

                <span className="mr-1">
                    {data.label || meta.title}
                </span>
            </div>

            <CustomHandle
                type="target"
                id={sourceHandleId}
                position={Position.Left}
                isConnectable={1}
            />
        </>
    );
}

// eslint-disable-next-line react-refresh/only-export-components
export const metadata: RegisterNodeMetadata<EndNodeData> = {
    type: NODE_TYPE,
    node: memo(EndNode),
    detail: {
        icon: "i-mynaui:stop",
        title: "End",
        description: "End the chatbot flow",
    },
    available: false,
    defaultData: {
        label: "End",
        deletable: false,
    },
};
