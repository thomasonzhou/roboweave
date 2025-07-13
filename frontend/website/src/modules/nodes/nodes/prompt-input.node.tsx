import { type Node, type NodeProps, Position } from "@xyflow/react";
import { nanoid } from "nanoid";
import { memo, useMemo, useState } from "react";

import CustomHandle from "~/modules/flow-builder/components/handles/custom-handle";
import { type BaseNodeData, BuilderNode, type RegisterNodeMetadata } from "~/modules/nodes/types";
import { getNodeDetail } from "~/modules/nodes/utils";

import { cn } from "~@/utils/cn";

export interface PromptInputNodeData extends BaseNodeData {
    label?: string;
    inputType?: "text" | "image" | "video";
    example?: string;
}

const NODE_TYPE = BuilderNode.PROMPT_INPUT;

type PromptInputNodeProps = NodeProps<Node<PromptInputNodeData, typeof NODE_TYPE>>;

export function PromptInputNode({ data, selected, isConnectable }: PromptInputNodeProps) {
    const meta = useMemo(() => getNodeDetail(NODE_TYPE), []);

    const [sourceHandleId] = useState<string>(nanoid());

    const getInputTypeIcon = (type: string) => {
        switch (type) {
            case "text": return "i-mynaui:chat";
            case "image": return "i-mynaui:image";
            case "video": return "i-mynaui:video";
            default: return "i-mynaui:chat";
        }
    };

    const getInputTypeColor = (type: string) => {
        switch (type) {
            case "text": return "bg-blue-500";
            case "image": return "bg-green-500";
            case "video": return "bg-purple-500";
            default: return "bg-blue-500";
        }
    };

    return (
        <>
            <div
                data-selected={selected}
                className="flex items-center border border-stone-300 rounded-xl bg-white px-4 py-3 shadow-sm transition data-[selected=true]:(border-orange-500 ring-1 ring-orange-500/50) min-w-48"
            >
                <div className={cn(
                    "rounded-lg p-2 mr-3",
                    getInputTypeColor(data.inputType || "text")
                )}>
                    <div className={cn(getInputTypeIcon(data.inputType || "text"), "size-4 text-white")} />
                </div>

                <div className="flex-1">
                    <div className="font-medium text-stone-800 text-sm">
                        {data.label || meta.title}
                    </div>
                    {data.example && (
                        <div className="text-xs text-stone-500 mt-1">
                            {data.example}
                        </div>
                    )}
                </div>
            </div>

            <CustomHandle
                type="source"
                id={sourceHandleId}
                position={Position.Right}
                isConnectable={isConnectable}
            />
        </>
    );
}

export const metadata: RegisterNodeMetadata<PromptInputNodeData> = {
    type: NODE_TYPE,
    node: memo(PromptInputNode),
    detail: {
        icon: "i-mynaui:chat",
        title: "Prompt Input",
        description: "Multimodal input for robot control",
    },
    defaultData: {
        label: "Prompt Input",
        inputType: "text",
        example: "Walk forward without hitting anything",
    },
}; 