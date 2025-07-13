import { nanoid } from "nanoid";

import type { Node } from "@xyflow/react";
import type { BuilderNodeType } from "~/modules/nodes/types";

import { NODES } from "~/modules/nodes";

export function getNodeDetail(nodeType: BuilderNodeType | string | undefined) {
    const node = NODES.find(node => node.type === nodeType);
    if (!node) throw new Error(`Node type "${nodeType}" not found`);

    return node.detail;
}

export function createNodeData<T extends BuilderNodeType>(type: T, data: any) {
    return {
        id: nanoid(),
        type,
        data,
    };
}

export function createNodeWithDefaultData(type: BuilderNodeType, data?: Partial<Node>) {
    const defaultData = NODES.find(node => node.type === type)?.defaultData;
    if (!defaultData) throw new Error(`No default data found for node type "${type}"`);
    return Object.assign(createNodeData(type, defaultData), data) as Node;
}
