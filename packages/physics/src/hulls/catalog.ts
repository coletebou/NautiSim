import type { Vessel } from '../core/types';
import { createKVLCC2 } from './kvlcc2';
import { createPlaningBoat, RIB_65, RUNABOUT_8 } from './planing';

/** The hulls offered in the app's hull picker. Builders are lazy: meshes are made on demand. */
export interface HullCatalogEntry {
  id: string;
  name: string;
  description: string;
  model: string;
  lengthM: number;
  build: (opts?: { resolution?: 'physics' | 'display' }) => Vessel;
}

export const HULL_CATALOG: readonly HullCatalogEntry[] = [
  {
    id: RIB_65.id,
    name: RIB_65.name,
    description: RIB_65.description,
    model: 'Savitsky planing + mesh hydrostatics',
    lengthM: RIB_65.length,
    build: (o) => createPlaningBoat(RIB_65, o),
  },
  {
    id: RUNABOUT_8.id,
    name: RUNABOUT_8.name,
    description: RUNABOUT_8.description,
    model: 'Savitsky planing + mesh hydrostatics',
    lengthM: RUNABOUT_8.length,
    build: (o) => createPlaningBoat(RUNABOUT_8, o),
  },
  {
    id: 'kvlcc2',
    name: 'KVLCC2 tanker 320 m',
    description: '320 m very large crude carrier. Takes kilometres to stop and minutes to turn; manoeuvring from the published MMG coefficients.',
    model: 'MMG standard method (Yasukawa & Yoshimura 2015)',
    lengthM: 320,
    build: (o) => createKVLCC2({ scale: 1, ...(o?.resolution !== undefined && { resolution: o.resolution }) }),
  },
];

export function findHull(id: string): HullCatalogEntry {
  const h = HULL_CATALOG.find((e) => e.id === id);
  if (!h) throw new Error(`unknown hull ${id}`);
  return h;
}
